#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Laser Illuminator – GUI‑приложение, реализующее протокол PELCO‑D
(см. Protocol_Laser_Illuminator.pdf).
"""

import sys
import socket
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QSlider, QGroupBox, QMessageBox,
    QSpinBox, QComboBox, QTextEdit, QLineEdit, QDial, QSizePolicy,
    QScrollArea, QFrame
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from PyQt5.QtCore import QPointF
import math


# ══════════════════════════════════════════════════════════════════════
#   Таймер дебаунса
# ══════════════════════════════════════════════════════════════════════
class DebounceTimer:
    """Таймер для дебаунсинга событий."""
    def __init__(self, delay_ms: int, callback):
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(callback)
        self.delay_ms = delay_ms

    def start(self):
        self.timer.stop()  # Перезапускаем таймер при каждом новом событии
        self.timer.start(self.delay_ms)

    def stop(self):
        self.timer.stop()


# ══════════════════════════════════════════════════════════════════════
#   Кастомные виджеты
# ══════════════════════════════════════════════════════════════════════

class BrightnessKnob(QDial):
    """Круглый регулятор яркости 0–255 с подписью значения."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(0, 255)
        self.setValue(128)
        self.setNotchesVisible(True)
        self.setFixedSize(90, 90)
        self.setStyleSheet("""
            QDial {
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                    fx:0.5, fy:0.5,
                    stop:0 #2d2d2d, stop:1 #1e1e1e);
                border: 2px solid #3c3c3c;
                border-radius: 45px;
            }
            QDial::handle {
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                    fx:0.5, fy:0.5,
                    stop:0 #dcdcdc, stop:1 #9cdcfe);
                border: 1px solid #3c3c3c;
                width: 15px;
                height: 15px;
                border-radius: 7px;
                margin: -7px;
            }
        """)


class AngleArcWidget(QWidget):
    """
    Визуализация угла светового конуса лазера.
    Рисует дугу, символизирующую раскрыв луча (1.8°–71°).
    """
    valueChanged = pyqtSignal(float)

    def __init__(self, min_deg=1.8, max_deg=71.0, parent=None):
        super().__init__(parent)
        self._min = min_deg
        self._max = max_deg
        self._value = 10.0          # текущий угол в градусах
        self.setFixedSize(200, 140)
        self.setMouseTracking(True)
        self._dragging = False

    @property
    def value(self):
        return self._value

    def setValue(self, deg: float):
        deg = max(self._min, min(self._max, deg))
        if abs(deg - self._value) > 0.05:
            self._value = deg
            self.update()
            self.valueChanged.emit(deg)

    # ---- рисование -------------------------------------------------------
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w // 2, h - 20          # вершина конуса внизу по центру

        half = self._value / 2.0
        length = 100

        # Лучи
        left_x  = cx - length * math.sin(math.radians(half))
        left_y  = cy - length * math.cos(math.radians(half))
        right_x = cx + length * math.sin(math.radians(half))
        right_y = cy - length * math.cos(math.radians(half))

        # Заливка конуса
        cone = QPolygonF([
            QPointF(cx, cy),
            QPointF(left_x, left_y),
            QPointF(right_x, right_y),
        ])
        p.setBrush(QBrush(QColor(80, 140, 255, 60)))
        p.setPen(Qt.NoPen)
        p.drawPolygon(cone)

        # Контур конуса
        pen = QPen(QColor(100, 160, 255), 2)
        p.setPen(pen)
        p.drawLine(int(cx), int(cy), int(left_x), int(left_y))
        p.drawLine(int(cx), int(cy), int(right_x), int(right_y))

        # Дуга
        arc_r = 60
        arc_rect_x = cx - arc_r
        arc_rect_y = cy - arc_r
        span_angle = int(self._value * 16)
        start_angle = int((90 - self._value / 2) * 16)
        pen2 = QPen(QColor(255, 200, 50), 2)
        p.setPen(pen2)
        p.drawArc(arc_rect_x, arc_rect_y, arc_r * 2, arc_r * 2,
                  start_angle, span_angle)

        # Точка вершины
        p.setBrush(QBrush(QColor(255, 100, 100)))
        p.setPen(Qt.NoPen)
        p.drawEllipse(cx - 4, cy - 4, 8, 8)

        # Текст угла
        p.setPen(QColor(220, 220, 255))
        font = QFont('Arial', 10, QFont.Bold)
        p.setFont(font)
        text = f'{self._value:.1f}°'
        p.drawText(0, 0, w, 22, Qt.AlignHCenter | Qt.AlignTop, text)

    # ---- мышь ------------------------------------------------------------
    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._drag_start_y = e.y()
            self._drag_start_val = self._value

    def mouseMoveEvent(self, e):
        if self._dragging:
            dy = self._drag_start_y - e.y()
            delta = dy * (self._max - self._min) / 120.0
            self.setValue(self._drag_start_val + delta)

    def mouseReleaseEvent(self, e):
        self._dragging = False

    def wheelEvent(self, e):
        delta = e.angleDelta().y() / 120.0
        self.setValue(self._value + delta)


class MotorPositionWidget(QWidget):
    """
    Круговой виджет позиции мотора.
    Показывает «стрелку» в диапазоне 0x0000–0x4000 (мин–макс хода мотора).
    """
    valueChanged = pyqtSignal(int)   # внутреннее значение 0–0x4000

    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 0x2000         # середина
        self._min = 0
        self._max = 0x4000
        self.setFixedSize(160, 160)
        self._dragging = False

    @property
    def value(self):
        return self._value

    def setValue(self, v: int):
        v = max(self._min, min(self._max, int(v)))
        if v != self._value:
            self._value = v
            self.update()
            self.valueChanged.emit(v)

    def _angle_for_value(self, v):
        """Отображаем 0–0x4000 на -135°…+135° (от нижнего левого до нижнего правого)."""
        frac = v / self._max
        return -135 + frac * 270

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        r = min(w, h) // 2 - 8

        # Фоновый круг
        p.setBrush(QBrush(QColor(30, 30, 50)))
        p.setPen(QPen(QColor(80, 80, 130), 2))
        p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r)

        # Шкала (засечки)
        p.setPen(QPen(QColor(100, 100, 160), 1))
        for i in range(0, 10):
            a = math.radians(-135 + i * 30)
            x1 = cx + (r - 6) * math.cos(a)
            y1 = cy + (r - 6) * math.sin(a)
            x2 = cx + r * math.cos(a)
            y2 = cy + r * math.sin(a)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

        # Дуга диапазона
        pen_arc = QPen(QColor(60, 120, 200), 4)
        p.setPen(pen_arc)
        p.setBrush(Qt.NoBrush)
        margin = 6
        p.drawArc(cx - r + margin, cy - r + margin,
                  2 * (r - margin), 2 * (r - margin),
                  int((-135 + 90) * 16), -int(270 * 16))   # Qt: против часовой

        # Стрелка
        angle_deg = self._angle_for_value(self._value)
        a = math.radians(angle_deg)
        needle_len = r - 14
        nx = cx + needle_len * math.cos(a)
        ny = cy + needle_len * math.sin(a)
        p.setPen(QPen(QColor(255, 80, 80), 3))
        p.drawLine(cx, cy, int(nx), int(ny))

        # Центральная точка
        p.setBrush(QBrush(QColor(200, 200, 255)))
        p.setPen(Qt.NoPen)
        p.drawEllipse(cx - 5, cy - 5, 10, 10)

        # Текст
        p.setPen(QColor(220, 220, 255))
        p.setFont(QFont('Arial', 8, QFont.Bold))
        pct = int(self._value / self._max * 100)
        p.drawText(0, cy + r - 12, w, 16, Qt.AlignHCenter, f'{pct}%  (0x{self._value:04X})')

    # ---- мышь ------------------------------------------------------------
    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._drag_start_x = e.x()
            self._drag_start_val = self._value

    def mouseMoveEvent(self, e):
        if self._dragging:
            dx = e.x() - self._drag_start_x
            delta = int(dx * self._max / 150)
            self.setValue(self._drag_start_val + delta)

    def mouseReleaseEvent(self, e):
        self._dragging = False

    def wheelEvent(self, e):
        delta = e.angleDelta().y() // 120
        self.setValue(self._value + delta * 0x100)


# ══════════════════════════════════════════════════════════════════════
#   Worker – отдельный поток для одной команды
# ══════════════════════════════════════════════════════════════════════
class Worker(QThread):
    responseReceived = pyqtSignal(bytes, bytes)
    errorOccurred    = pyqtSignal(str)

    def __init__(self, connection, packet: bytes,
                 timeout: float = 5.0, parent=None, conn_type='tcp'):
        super().__init__(parent)
        self.connection = connection
        self.packet     = packet
        self.timeout    = timeout
        self.conn_type  = conn_type

    def _recv_full_tcp(self, size: int) -> bytes:
        data = b''
        self.connection.settimeout(self.timeout)
        while len(data) < size:
            chunk = self.connection.recv(size - len(data))
            if not chunk:
                raise ConnectionError('Сокет закрыт удалённой стороной')
            data += chunk
        return data

    def _recv_full_serial(self, size: int) -> bytes:
        data = b''
        received = 0
        max_attempts = 20
        attempts = 0
        while received < size:
            chunk = self.connection.read(size - received)
            if not chunk:
                attempts += 1
                if attempts >= max_attempts:
                    raise TimeoutError(f'Таймаут: получено {received} из {size} байт')
                continue
            attempts = 0
            data += chunk
            received += len(chunk)
        return data

    def run(self):
        try:
            if self.conn_type == 'tcp':
                self.connection.sendall(self.packet)
            else:
                self.connection.write(self.packet)

            cmd1, cmd2 = self.packet[2], self.packet[3]
            if (cmd1, cmd2) in LaserController.REQUEST_CMDS:
                if self.conn_type == 'tcp':
                    resp = self._recv_full_tcp(7)
                else:
                    resp = self._recv_full_serial(7)
                self.responseReceived.emit(resp, self.packet)
            else:
                self.responseReceived.emit(b'', self.packet)
        except Exception as exc:
            self.errorOccurred.emit(str(exc))


# ══════════════════════════════════════════════════════════════════════
#   Главное окно
# ══════════════════════════════════════════════════════════════════════
class LaserController(QWidget):

    REQUEST_CMDS = {
        (0x02, 0x01),   # запрос статуса питания
        (0x02, 0x03),   # запрос яркости
        (0x02, 0x05),   # запрос позиции мотора
        (0x02, 0x0F),   # запрос статуса вентилятора
        (0x09, 0x01),   # запрос угла света
        (0x05, 0x10),   # запрос версии ПО
    }

    def __init__(self):
        super().__init__()
        self.tcp_host      = '192.168.0.7'
        self.tcp_port      = 20108
        self.serial_port   = None
        self.serial_baud   = 9600
        self.connection    = None
        self.conn_type     = 'tcp'
        self.device_address = 0x01
        self._workers      = []

        # Таймеры дебаунса для автоматического применения изменений
        self.br_debounce_timer = DebounceTimer(300, self.set_brightness)  # 300мс задержка для яркости
        self.spot_debounce_timer = DebounceTimer(500, self.set_spot_angle)  # 500мс задержка для угла
        self.motor_debounce_timer = DebounceTimer(500, self.set_motor_angle)  # 500мс задержка для мотора

        self.init_ui()
        self._set_controls_enabled(False)   # кнопки заблокированы до подключения

    # ──────────────────────────────────────────────────────────────────
    #   Блокировка / разблокировка управляющих групп
    # ──────────────────────────────────────────────────────────────────
    def _set_controls_enabled(self, enabled: bool):
        for w in self._control_widgets:
            w.setEnabled(enabled)

    # ──────────────────────────────────────────────────────────────────
    #   UI
    # ──────────────────────────────────────────────────────────────────
    def init_ui(self):
        self.setWindowTitle('Управление лазерной подсветкой')
        self.setMinimumWidth(560)
        self.setStyleSheet("""
            QWidget       { background:#1e1e1e; color:#d4d4d4; font-size:13px; }
            QGroupBox     { border:1px solid #3c3c3c; border-radius:6px;
                            margin-top:10px; padding-top:6px; font-weight:bold; color:#9cdcfe; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; top:-2px; }
            QPushButton   { background:#3c3c3c; border:1px solid #3c3c3c;
                            border-radius:5px; padding:5px 12px; color:#dcdcdc; }
            QPushButton:hover   { background:#4c4c4c; }
            QPushButton:pressed { background:#2a2a2a; }
            QPushButton:disabled{ background:#2d2d2d; color:#6a6a6a;
                                  border-color:#3c3c3c; }
            QLineEdit, QComboBox, QSpinBox {
                background:#3c3c3c; border:1px solid #3c3c3c;
                border-radius:4px; padding:3px 6px; color:#dcdcdc; selection-background-color:#264f78; }
            QTextEdit     { background:#1e1e1e; border:1px solid #3c3c3c;
                            border-radius:4px; color:#9cdcfe; font-family:monospace; }
            QSlider::groove:horizontal { height:6px; background:#2d2d2d;
                border-radius:3px; }
            QSlider::handle:horizontal { width:16px; height:16px; margin:-5px 0;
                background:#dcdcdc; border-radius:8px; }
            QSlider::sub-page:horizontal { background:#007acc; border-radius:3px; }
            QLabel#lbl_status_connected { color:#89d185; font-weight:bold; }
            QLabel#lbl_status_disconnected { color:#f48771; font-weight:bold; }
            QDial {
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                    fx:0.5, fy:0.5,
                    stop:0 #2d2d2d, stop:1 #1e1e1e);
                border: 2px solid #3c3c3c;
                border-radius: 45px;
            }
            QDial::handle {
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.5,
                    fx:0.5, fy:0.5,
                    stop:0 #dcdcdc, stop:1 #9cdcfe);
                border: 1px solid #3c3c3c;
                width: 15px;
                height: 15px;
                border-radius: 7px;
                margin: -7px;
            }
        """)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        inner = QWidget()
        main = QVBoxLayout(inner)
        main.setSpacing(8)
        main.setContentsMargins(10, 10, 10, 10)

        # ---- запоминаем все управляющие виджеты для enable/disable ----
        self._control_widgets = []

        # ══ 1. Подключение ══════════════════════════════════════════════
        group_conn = QGroupBox('Подключение')
        lay_conn = QGridLayout()
        lay_conn.setSpacing(6)

        self.cmb_conn_type = QComboBox()
        self.cmb_conn_type.addItem('TCP/IP', 'tcp')
        self.cmb_conn_type.addItem('COM-порт', 'serial')
        self.cmb_conn_type.currentTextChanged.connect(self.on_conn_type_changed)

        self.lbl_host  = QLabel('IP-адрес:')
        self.edit_host = QLineEdit(self.tcp_host)
        self.lbl_port  = QLabel('Порт:')
        self.edit_port = QLineEdit(str(self.tcp_port))

        self.lbl_serial_port  = QLabel('COM-порт:')
        self.cmb_serial_port  = QComboBox()
        self.refresh_serial_ports()
        self.btn_refresh_ports = QPushButton('Обновить')
        self.btn_refresh_ports.clicked.connect(self.refresh_serial_ports)
        self.lbl_baud = QLabel('Скорость:')
        self.cmb_baud = QComboBox()
        for baud in [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]:
            self.cmb_baud.addItem(str(baud), baud)
        self.cmb_baud.setCurrentText(str(self.serial_baud))

        self.lbl_status = QLabel('● Не подключено')
        self.lbl_status.setObjectName('lbl_status_disconnected')
        self.btn_conn = QPushButton('Подключиться')
        self.btn_conn.clicked.connect(self.toggle_connection)
        self.btn_conn.setStyleSheet(
            'QPushButton { background:#007acc; border-color:#007acc; color:white; }'
            'QPushButton:hover { background:#005a9e; }'
        )

        lay_conn.addWidget(QLabel('Тип:'),         0, 0)
        lay_conn.addWidget(self.cmb_conn_type,     0, 1, 1, 2)
        lay_conn.addWidget(self.lbl_host,          1, 0)
        lay_conn.addWidget(self.edit_host,         1, 1, 1, 2)
        lay_conn.addWidget(self.lbl_port,          2, 0)
        lay_conn.addWidget(self.edit_port,         2, 1, 1, 2)
        lay_conn.addWidget(self.lbl_serial_port,   3, 0)
        lay_conn.addWidget(self.cmb_serial_port,   3, 1)
        lay_conn.addWidget(self.btn_refresh_ports, 3, 2)
        lay_conn.addWidget(self.lbl_baud,          4, 0)
        lay_conn.addWidget(self.cmb_baud,          4, 1, 1, 2)
        lay_conn.addWidget(self.lbl_status,        5, 0, 1, 2)
        lay_conn.addWidget(self.btn_conn,          5, 2)

        group_conn.setLayout(lay_conn)
        main.addWidget(group_conn)
        self.update_connection_fields()

        # ══ 2. Питание ══════════════════════════════════════════════════
        group_power = QGroupBox('Питание лазера')
        lay_power = QHBoxLayout()

        self.btn_on  = QPushButton('⚡  Включить')
        self.btn_off = QPushButton('○  Выключить')
        self.btn_qp  = QPushButton('?  Статус')
        self.btn_on.setStyleSheet(
            'QPushButton{background:#388A34;border-color:#388A34;color:white;font-weight:bold;}'
            'QPushButton:hover{background:#3C9D3A;}'
            'QPushButton:disabled{background:#2d2d2d;color:#6a6a6a;border-color:#3c3c3c;}'
        )
        self.btn_off.setStyleSheet(
            'QPushButton{background:#BE1100;border-color:#BE1100;color:white;font-weight:bold;}'
            'QPushButton:hover{background:#CE392F;}'
            'QPushButton:disabled{background:#2d2d2d;color:#6a6a6a;border-color:#3c3c3c;}'
        )
        self.btn_on.clicked.connect(self.power_on)
        self.btn_off.clicked.connect(self.power_off)
        self.btn_qp.clicked.connect(self.query_power)

        lay_power.addWidget(self.btn_on)
        lay_power.addWidget(self.btn_off)
        lay_power.addWidget(self.btn_qp)
        group_power.setLayout(lay_power)
        main.addWidget(group_power)
        self._control_widgets += [self.btn_on, self.btn_off, self.btn_qp]

        # ══ 3. Яркость ══════════════════════════════════════════════════
        group_br = QGroupBox('Яркость')
        lay_br = QVBoxLayout()

        # Верхняя строка: метка + запрос
        row_br_top = QHBoxLayout()
        self.lbl_cur_br = QLabel('Текущая: —')
        self.btn_qb = QPushButton('Запросить')
        self.btn_qb.clicked.connect(self.query_brightness)
        row_br_top.addWidget(self.lbl_cur_br)
        row_br_top.addStretch()
        row_br_top.addWidget(self.btn_qb)

        # Knob + значение (без кнопки, т.к. изменения применяются автоматически с дебаунсом)
        row_br_ctrl = QHBoxLayout()
        row_br_ctrl.setAlignment(Qt.AlignVCenter)

        self.dial_br = BrightnessKnob()
        self.lbl_br_val = QLabel('128')
        self.lbl_br_val.setAlignment(Qt.AlignCenter)
        self.lbl_br_val.setStyleSheet('font-size:22px; font-weight:bold; color:#9cdcfe; min-width:50px;')
        self.lbl_br_pct = QLabel('50%')
        self.lbl_br_pct.setAlignment(Qt.AlignCenter)
        self.lbl_br_pct.setStyleSheet('color:#d4d4d4; font-size:11px;')

        self.dial_br.valueChanged.connect(self._on_dial_br_changed)

        val_col = QVBoxLayout()
        val_col.addWidget(self.lbl_br_val)
        val_col.addWidget(self.lbl_br_pct)

        row_br_ctrl.addStretch()
        row_br_ctrl.addWidget(self.dial_br)
        row_br_ctrl.addSpacing(12)
        row_br_ctrl.addLayout(val_col)
        row_br_ctrl.addStretch()

        lay_br.addLayout(row_br_top)
        lay_br.addLayout(row_br_ctrl)
        group_br.setLayout(lay_br)
        main.addWidget(group_br)
        self._control_widgets += [self.dial_br, self.btn_qb]  # Убрали btn_set_br из списка

        # ══ 4. Угол светового пятна ═════════════════════════════════════
        group_spot = QGroupBox('Угол светового пятна (1.8° – 71°)')
        lay_spot = QVBoxLayout()

        row_spot_top = QHBoxLayout()
        self.lbl_cur_spot = QLabel('Текущий: —')
        self.btn_q_spot = QPushButton('Запросить')
        self.btn_q_spot.clicked.connect(self.query_spot_angle)
        row_spot_top.addWidget(self.lbl_cur_spot)
        row_spot_top.addStretch()
        row_spot_top.addWidget(self.btn_q_spot)

        row_spot_ctrl = QHBoxLayout()
        row_spot_ctrl.setAlignment(Qt.AlignVCenter)

        self.arc_widget = AngleArcWidget(1.8, 71.0)
        self.arc_widget.setValue(10.0)
        self.arc_widget.valueChanged.connect(self._on_arc_changed)

        spot_btn_col = QVBoxLayout()
        self.btn_inc = QPushButton('▲ Увеличить')
        self.btn_dec = QPushButton('▼ Уменьшить')
        self.btn_set_spot = QPushButton('✔ Установить')
        self.btn_inc.clicked.connect(self.increase_angle)
        self.btn_dec.clicked.connect(self.decrease_angle)
        self.btn_set_spot.clicked.connect(self.set_spot_angle)
        spot_btn_col.addWidget(self.btn_inc)
        spot_btn_col.addWidget(self.btn_dec)
        spot_btn_col.addWidget(self.btn_set_spot)

        row_spot_ctrl.addStretch()
        row_spot_ctrl.addWidget(self.arc_widget)
        row_spot_ctrl.addSpacing(16)
        row_spot_ctrl.addLayout(spot_btn_col)
        row_spot_ctrl.addStretch()

        lay_spot.addLayout(row_spot_top)
        lay_spot.addLayout(row_spot_ctrl)
        group_spot.setLayout(lay_spot)
        main.addWidget(group_spot)
        self._control_widgets += [
            self.btn_q_spot, self.arc_widget,
            self.btn_inc, self.btn_dec, self.btn_set_spot
        ]

        # ══ 5. Позиция мотора ═══════════════════════════════════════════
        group_motor = QGroupBox('Позиция мотора (0x0000 – 0x4000)')
        lay_motor = QVBoxLayout()

        row_motor_top = QHBoxLayout()
        self.lbl_cur_motor = QLabel('Текущая: —')
        self.btn_q_motor = QPushButton('Запросить')
        self.btn_q_motor.clicked.connect(self.query_motor_angle)
        row_motor_top.addWidget(self.lbl_cur_motor)
        row_motor_top.addStretch()
        row_motor_top.addWidget(self.btn_q_motor)

        row_motor_ctrl = QHBoxLayout()
        row_motor_ctrl.setAlignment(Qt.AlignVCenter)

        self.motor_widget = MotorPositionWidget()
        self.motor_widget.setValue(0x2000)
        self.motor_widget.valueChanged.connect(self._on_motor_widget_changed)

        motor_btn_col = QVBoxLayout()
        self.btn_set_motor = QPushButton('✔ Установить')
        self.btn_reset_motor = QPushButton('⟳ Сброс мотора')
        self.btn_set_motor.clicked.connect(self.set_motor_angle)
        self.btn_reset_motor.clicked.connect(self.reset_motor)
        motor_btn_col.addWidget(self.btn_set_motor)
        motor_btn_col.addWidget(self.btn_reset_motor)

        row_motor_ctrl.addStretch()
        row_motor_ctrl.addWidget(self.motor_widget)
        row_motor_ctrl.addSpacing(16)
        row_motor_ctrl.addLayout(motor_btn_col)
        row_motor_ctrl.addStretch()

        lay_motor.addLayout(row_motor_top)
        lay_motor.addLayout(row_motor_ctrl)
        group_motor.setLayout(lay_motor)
        main.addWidget(group_motor)
        self._control_widgets += [
            self.btn_q_motor, self.motor_widget,
            self.btn_set_motor, self.btn_reset_motor
        ]

        # ══ 6. Доп. функции ═════════════════════════════════════════════
        group_misc = QGroupBox('Дополнительно')
        grid_misc = QGridLayout()
        grid_misc.setSpacing(6)

        self.btn_fan_q = QPushButton('Статус вентилятора')
        self.btn_ver_q = QPushButton('Версия ПО')
        self.btn_fan_q.clicked.connect(self.query_fan)
        self.btn_ver_q.clicked.connect(self.query_version)

        self.cmb_addr = QComboBox()
        for i in range(1, 255):
            self.cmb_addr.addItem(str(i))
        self.cmb_addr.setCurrentText('1')
        self.btn_addr_set = QPushButton('Изменить адрес')
        self.btn_addr_set.clicked.connect(self.change_address)

        self.cmb_dev_baud = QComboBox()
        self.baud_map = {
            0x00: 1200, 0x01: 2400, 0x02: 4800, 0x03: 9600,
            0x04: 19200, 0x05: 38400, 0x06: 57600, 0x07: 115200
        }
        for code, rate in self.baud_map.items():
            self.cmb_dev_baud.addItem(str(rate), code)
        self.cmb_dev_baud.setCurrentIndex(3)
        self.btn_baud_set = QPushButton('Изменить скорость')
        self.btn_baud_set.clicked.connect(self.change_baud)

        grid_misc.addWidget(self.btn_fan_q,       0, 0)
        grid_misc.addWidget(self.btn_ver_q,       0, 1)
        grid_misc.addWidget(QLabel('Адрес устр.:'),1, 0)
        grid_misc.addWidget(self.cmb_addr,        1, 1)
        grid_misc.addWidget(self.btn_addr_set,    1, 2)
        grid_misc.addWidget(QLabel('Baud устр.:'), 2, 0)
        grid_misc.addWidget(self.cmb_dev_baud,    2, 1)
        grid_misc.addWidget(self.btn_baud_set,    2, 2)

        group_misc.setLayout(grid_misc)
        main.addWidget(group_misc)
        self._control_widgets += [
            self.btn_fan_q, self.btn_ver_q,
            self.cmb_addr, self.btn_addr_set,
            self.cmb_dev_baud, self.btn_baud_set,
        ]

        # ══ 7. Журнал ═══════════════════════════════════════════════════
        group_log = QGroupBox('Журнал')
        vlog = QVBoxLayout()
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setFixedHeight(130)
        vlog.addWidget(self.log_edit)
        group_log.setLayout(vlog)
        main.addWidget(group_log)

        scroll.setWidget(inner)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    # ──────────────────────────────────────────────────────────────────
    #   Обработчики кастомных виджетов
    # ──────────────────────────────────────────────────────────────────
    def _on_dial_br_changed(self, v):
        self.lbl_br_val.setText(str(v))
        self.lbl_br_pct.setText(f'{int(v / 255 * 100)}%')
        # Запускаем таймер дебаунса для автоматического применения изменения яркости
        self.br_debounce_timer.start()

    def set_brightness_auto(self):
        """Автоматическое применение изменения яркости с дебаунсом"""
        val = self.dial_br.value()
        self.send_packet(self.build_packet(0x01, 0x03, val, 0x00), timeout=2)

    def _on_arc_changed(self, deg):
        self.lbl_cur_spot.setText(f'Выбрано: {deg:.1f}°')
        # Запускаем таймер дебаунса для автоматического применения изменения угла
        self.spot_debounce_timer.start()

    def set_spot_angle_auto(self):
        """Автоматическое применение изменения угла света с дебаунсом"""
        # Конвертируем градусы → внутреннее значение для команды 0x08 0x01
        deg = self.arc_widget.value
        internal = int(round((71 - deg) / (71 - 1.8) * 0x4000))
        internal = max(0, min(0x4000, internal))
        hi, lo = (internal >> 8) & 0xFF, internal & 0xFF
        self.send_packet(self.build_packet(0x08, 0x01, hi, lo), timeout=5)

    def _on_motor_widget_changed(self, v):
        self.lbl_cur_motor.setText(f'Выбрано: 0x{v:04X}')
        # Запускаем таймер дебаунса для автоматического применения изменения мотора
        self.motor_debounce_timer.start()

    def set_motor_angle_auto(self):
        """Автоматическое применение изменения позиции мотора с дебаунсом"""
        ang = self.motor_widget.value
        hi, lo = (ang >> 8) & 0xFF, ang & 0xFF
        self.send_packet(self.build_packet(0x01, 0x05, hi, lo), timeout=5)

    # ──────────────────────────────────────────────────────────────────
    #   Пакет-утилиты
    # ──────────────────────────────────────────────────────────────────
    def _calc_checksum(self, data: list) -> int:
        return sum(data) & 0xFF

    def build_packet(self, cmd1, cmd2, data1=0, data2=0) -> bytes:
        pkt = [0xFF, self.device_address, cmd1, cmd2, data1, data2]
        pkt.append(self._calc_checksum(pkt[1:]))
        return bytes(pkt)

    def send_packet(self, packet: bytes, timeout: float = 5.0):
        if not self.connection:
            QMessageBox.warning(self, 'Нет подключения', 'Сначала подключитесь к лазеру!')
            return
        worker = Worker(self.connection, packet, timeout, conn_type=self.conn_type)
        self._workers.append(worker)
        worker.finished.connect(lambda w=worker: self._workers.remove(w) if w in self._workers else None)
        worker.responseReceived.connect(self._on_response)
        worker.errorOccurred.connect(self._on_error)
        worker.start()

    # ──────────────────────────────────────────────────────────────────
    #   Слоты от Worker
    # ──────────────────────────────────────────────────────────────────
    @pyqtSlot(bytes, bytes)
    def _on_response(self, response: bytes, request: bytes):
        if not response:
            self.log_message(f'Отправлено: {self._hex(request)}')
            return
        self.log_message(f'Ответ: {self._hex(response)}')
        self.process_response(response, request)

    @pyqtSlot(str)
    def _on_error(self, msg: str):
        self.log_message(f'Ошибка: {msg}')
        QMessageBox.critical(self, 'Ошибка соединения', msg)

    # ──────────────────────────────────────────────────────────────────
    #   Лог
    # ──────────────────────────────────────────────────────────────────
    def _hex(self, data: bytes) -> str:
        return ' '.join(f'{b:02X}' for b in data)

    def log_message(self, text: str):
        self.log_edit.append(text)
        if self.log_edit.document().blockCount() > 500:
            cursor = self.log_edit.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    # ──────────────────────────────────────────────────────────────────
    #   Подключение
    # ──────────────────────────────────────────────────────────────────
    def on_conn_type_changed(self):
        self.conn_type = self.cmb_conn_type.currentData()
        self.update_connection_fields()

    def update_connection_fields(self):
        is_tcp = self.conn_type == 'tcp'
        for w in [self.lbl_host, self.edit_host, self.lbl_port, self.edit_port]:
            w.setVisible(is_tcp)
        for w in [self.lbl_serial_port, self.cmb_serial_port,
                  self.btn_refresh_ports, self.lbl_baud, self.cmb_baud]:
            w.setVisible(not is_tcp)

    def refresh_serial_ports(self):
        self.cmb_serial_port.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        for p in ports:
            self.cmb_serial_port.addItem(p)

    def toggle_connection(self):
        if self.connection is None:
            if self.connect_to_laser():
                self.lbl_status.setText('● Подключено')
                self.lbl_status.setObjectName('lbl_status_connected')
                self.lbl_status.setStyleSheet('color:#89d185; font-weight:bold;')
                self.btn_conn.setText('Отключиться')
                self.btn_conn.setStyleSheet(
                    'QPushButton{background:#BE1100;border-color:#BE1100;color:white;font-weight:bold;}'
                    'QPushButton:hover{background:#CE392F;}'
                )
                self._set_controls_enabled(True)
        else:
            self.disconnect_from_laser()
            self.lbl_status.setText('● Не подключено')
            self.lbl_status.setStyleSheet('color:#f48771; font-weight:bold;')
            self.btn_conn.setText('Подключиться')
            self.btn_conn.setStyleSheet(
                'QPushButton { background:#007acc; border-color:#007acc; color:white; }'
                'QPushButton:hover { background:#005a9e; }'
            )
            self._set_controls_enabled(False)

    def connect_to_laser(self) -> bool:
        try:
            if self.conn_type == 'tcp':
                self.tcp_host = self.edit_host.text()
                self.tcp_port = int(self.edit_port.text())
                self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.connection.settimeout(10)
                self.connection.connect((self.tcp_host, self.tcp_port))
                self.log_message(f'TCP подключение: {self.tcp_host}:{self.tcp_port}')
            else:
                self.serial_port = self.cmb_serial_port.currentText()
                self.serial_baud = int(self.cmb_baud.currentText())
                self.connection = serial.Serial(self.serial_port, self.serial_baud, timeout=10)
                self.log_message(f'COM подключение: {self.serial_port} @ {self.serial_baud}')
            return True
        except Exception as exc:
            QMessageBox.critical(self, 'Ошибка подключения',
                                 f'Не удалось подключиться через {self.conn_type.upper()}.\n{exc}')
            self.connection = None
            return False

    def disconnect_from_laser(self):
        if self.connection:
            try:
                self.connection.close()
                self.log_message('Соединение закрыто')
            finally:
                self.connection = None

    # ──────────────────────────────────────────────────────────────────
    #   Команды
    # ──────────────────────────────────────────────────────────────────
    def power_on(self):
        self.send_packet(self.build_packet(0x01, 0x01, 0x01, 0x00), timeout=2)

    def power_off(self):
        self.send_packet(self.build_packet(0x01, 0x01, 0x00, 0x00), timeout=2)

    def query_power(self):
        self.send_packet(self.build_packet(0x02, 0x01, 0x00, 0x00), timeout=5)

    def set_brightness(self):
        val = self.dial_br.value()
        self.send_packet(self.build_packet(0x01, 0x03, val, 0x00), timeout=2)

    def set_brightness_auto(self):
        self.set_brightness()

    def query_brightness(self):
        self.send_packet(self.build_packet(0x02, 0x03, 0x00, 0x00), timeout=5)

    def set_spot_angle(self):
        # Конвертируем градусы → внутреннее значение для команды 0x08 0x01
        deg = self.arc_widget.value
        internal = int(round((71 - deg) / (71 - 1.8) * 0x4000))
        internal = max(0, min(0x4000, internal))
        hi, lo = (internal >> 8) & 0xFF, internal & 0xFF
        self.send_packet(self.build_packet(0x08, 0x01, hi, lo), timeout=5)

    def set_spot_angle_auto(self):
        self.set_spot_angle()

    def query_spot_angle(self):
        self.send_packet(self.build_packet(0x09, 0x01, 0x00, 0x00), timeout=5)

    def set_motor_angle(self):
        ang = self.motor_widget.value
        hi, lo = (ang >> 8) & 0xFF, ang & 0xFF
        self.send_packet(self.build_packet(0x01, 0x05, hi, lo), timeout=5)

    def set_motor_angle_auto(self):
        self.set_motor_angle()

    def query_motor_angle(self):
        self.send_packet(self.build_packet(0x02, 0x05, 0x00, 0x00), timeout=5)

    def increase_angle(self):
        # Останавливаем таймер дебаунса, чтобы избежать дублирования команды
        self.spot_debounce_timer.stop()
        self.send_packet(self.build_packet(0x01, 0x04, 0x01, 0x00), timeout=2)

    def decrease_angle(self):
        # Останавливаем таймер дебаунса, чтобы избежать дублирования команды
        self.spot_debounce_timer.stop()
        self.send_packet(self.build_packet(0x01, 0x04, 0x00, 0x00), timeout=2)

    def reset_motor(self):
        # Останавливаем таймер дебаунса, чтобы избежать дублирования команды
        self.motor_debounce_timer.stop()
        self.send_packet(self.build_packet(0x01, 0x06, 0x00, 0x00), timeout=15)

    def query_fan(self):
        self.send_packet(self.build_packet(0x02, 0x0F, 0x00, 0x00), timeout=5)

    def query_version(self):
        self.send_packet(self.build_packet(0x05, 0x10, 0x01, 0x01), timeout=5)

    def change_address(self):
        new_addr = int(self.cmb_addr.currentText())
        self.send_packet(self.build_packet(0x03, 0x11, 0x00, new_addr), timeout=5)

    def change_baud(self):
        code = self.cmb_dev_baud.currentData()
        self.send_packet(self.build_packet(0x03, 0x13, 0x00, code), timeout=5)

    # ──────────────────────────────────────────────────────────────────
    #   Обработка ответов
    # ──────────────────────────────────────────────────────────────────
    def process_response(self, resp: bytes, request: bytes):
        if len(resp) != 7:
            self.log_message('Ответ: неверный размер')
            return
        hdr, addr, cmd1, cmd2, d1, d2, chk = resp
        if hdr != 0xFF:
            self.log_message('Ответ: некорректный заголовок')
            return
        if addr != self.device_address:
            self.log_message(f'Ответ: чужой адрес {addr:#02x}')
            return
        if self._calc_checksum(list(resp[1:6])) != chk:
            self.log_message('Ответ: неверная контрольная сумма')
            return

        if (cmd1, cmd2) == (0x02, 0x01):
            if d1 == 0x00:
                self.lbl_status.setText('● Подключено  (Лазер ВЫКЛ)')
                self.lbl_status.setStyleSheet('color:#ffcc00; font-weight:bold;')
                self.log_message('Лазер: ВЫКЛ')
            elif d1 == 0x01:
                self.lbl_status.setText('● Подключено  (Лазер ВКЛ)')
                self.lbl_status.setStyleSheet('color:#89d185; font-weight:bold;')
                self.log_message('Лазер: ВКЛ')
            else:
                self.log_message(f'Статус питания: неизвестен ({d1:#02x})')

        elif (cmd1, cmd2) == (0x02, 0x03):
            self.dial_br.blockSignals(True)
            self.dial_br.setValue(d1)
            self.dial_br.blockSignals(False)
            self.lbl_br_val.setText(str(d1))
            self.lbl_br_pct.setText(f'{int(d1/255*100)}%')
            self.lbl_cur_br.setText(f'Текущая: {d1}')
            self.log_message(f'Яркость: {d1}')

        elif (cmd1, cmd2) == (0x02, 0x05):
            ang = (d1 << 8) | d2
            self.motor_widget.blockSignals(True)
            self.motor_widget.setValue(ang)
            self.motor_widget.blockSignals(False)
            self.lbl_cur_motor.setText(f'Текущая: 0x{ang:04X}')
            self.log_message(f'Позиция мотора: 0x{ang:04X}')

        elif (cmd1, cmd2) == (0x02, 0x0F):
            self.log_message(f'Вентилятор: {"ВКЛ" if d1 == 0x01 else "ВЫКЛ"}')

        elif (cmd1, cmd2) == (0x09, 0x01):
            ang = (d1 << 8) | d2
            # Протокол: единица 0.01°, поэтому делим на 100
            deg = ang * 0.01
            deg = max(1.8, min(71.0, deg))
            self.arc_widget.blockSignals(True)
            self.arc_widget.setValue(deg)
            self.arc_widget.blockSignals(False)
            self.lbl_cur_spot.setText(f'Текущий: {deg:.1f}°')
            self.log_message(f'Угол пятна: {deg:.2f}°')

        elif (cmd1, cmd2) == (0x20, 0x00):
            self.log_message(f'Версия ПО: {d1}.{d2}')

        elif (cmd1, cmd2) == (0x21, 0x00):
            self.log_message(f'Дата сборки: {2000+d1}-{d2:02d}')

        elif (cmd1, cmd2) == (0x22, 0x00):
            self.log_message(f'Тип модуля: 0x{(d1<<8)|d2:04X}')

        else:
            self.log_message(f'Необработанный ответ: {self._hex(resp)}')


# ══════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    w = LaserController()
    w.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()