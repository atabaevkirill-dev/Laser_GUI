#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Laser Illuminator – GUI‑приложение, реализующее протокол PELCO‑D
(см. Protocol _Laser Illuminator.pdf).

* В каждый пакет добавлен **адрес устройства** (по умолчанию 0x01);
* Контрольная сумма считается правильно;
* Отправка/приём выполняются в отдельном QThread → UI не «зависает»;
* Ответы читаются полностью (7 байт) и проверяются;
* Рабочие потоки хранятся в списке `self._workers`, чтобы они не
  уничожались преждевременно.
"""

import sys
import socket
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QSlider, QGroupBox, QMessageBox,
    QSpinBox, QComboBox, QTextEdit, QLineEdit
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot


# ----------------------------------------------------------------------
#   Worker – отдельный поток для единственной команды
# ----------------------------------------------------------------------
class Worker(QThread):
    """
    Отправляет один пакет и (если это запрос) ждёт 7‑байтовый ответ.
    По окончании испускает сигналы:
        responseReceived(bytes response, bytes request)
        errorOccurred(str message)
    """
    responseReceived = pyqtSignal(bytes, bytes)
    errorOccurred    = pyqtSignal(str)

    def __init__(self, connection, packet: bytes,
                 timeout: float = 5.0, parent=None, conn_type='tcp'):
        super().__init__(parent)
        self.connection = connection
        self.packet  = packet
        self.timeout = timeout
        self.conn_type = conn_type

    # ------------------------------------------------------------------
    def _recv_full_tcp(self, size: int) -> bytes:
        """Получить ровно `size` байт по TCP (TCP‑поток может отдать их частями)."""
        data = b''
        self.connection.settimeout(self.timeout)
        while len(data) < size:
            chunk = self.connection.recv(size - len(data))
            if not chunk:
                raise ConnectionError('Сокет закрыт удалённой стороной')
            data += chunk
        return data

    # ------------------------------------------------------------------
    def _recv_full_serial(self, size: int) -> bytes:
        """Получить ровно `size` байт по COM-порту."""
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

    # ------------------------------------------------------------------
    def run(self):
        try:
            # ---- отправка ------------------------------------------------
            if self.conn_type == 'tcp':
                self.connection.sendall(self.packet)
            else:  # serial
                self.connection.write(self.packet)

            # ---- нужно ли ждать ответ? ----------------------------------
            cmd1, cmd2 = self.packet[2], self.packet[3]
            if (cmd1, cmd2) in LaserController.REQUEST_CMDS:
                if self.conn_type == 'tcp':
                    resp = self._recv_full_tcp(7)          # ожидаем ровно 7 байт
                else:
                    resp = self._recv_full_serial(7)       # ожидаем ровно 7 байт по COM
                self.responseReceived.emit(resp, self.packet)
            else:
                # «тихие» команды – просто сообщаем об успехе
                self.responseReceived.emit(b'', self.packet)

        except Exception as exc:
            self.errorOccurred.emit(str(exc))


# ----------------------------------------------------------------------
#   Главное окно
# ----------------------------------------------------------------------
class LaserController(QWidget):
    """
    Всё, что относится к UI, а работа с сетью делегируется Worker‑у.
    """
    # --------------------------------------------------------------
    #  Список запросов, от которых ожидается ответ (пакет длиной 7 байт)
    # --------------------------------------------------------------
    REQUEST_CMDS = {
        (0x02, 0x01),   # запрос статуса питания
        (0x02, 0x03),   # запрос яркости
        (0x02, 0x05),   # запрос позиции мотора
        (0x02, 0x0F),   # запрос статуса вентилятора
        (0x09, 0x01),   # запрос угла света
        (0x05, 0x10),   # запрос версии ПО
    }

    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__()
        # Настройки подключения
        self.tcp_host = '192.168.0.7'
        self.tcp_port = 20108
        self.serial_port = None
        self.serial_baud = 9600  # значение по умолчанию

        self.connection = None                 # будет установлен после connect()
        self.conn_type = 'tcp'                 # по умолчанию TCP, может быть 'serial'
        self.device_address = 0x01             # по умолчанию, изменяется UI‑командой

        # хранить ссылки на запущенные потоки, чтобы они не удалялись
        self._workers = []

        self.init_ui()

    # ------------------------------------------------------------------
    # ------------------------------ UI --------------------------------
    # ------------------------------------------------------------------
    def init_ui(self):
        self.setWindowTitle('Управление лазерной подсветкой')
        self.setGeometry(100, 100, 500, 500)

        main = QVBoxLayout()

        # ---------- 1. Подключение ----------
        group_conn = QGroupBox('Настройки подключения')
        lay_conn = QGridLayout()
        
        # Выбор типа подключения
        self.lbl_conn_type = QLabel('Тип подключения:')
        self.cmb_conn_type = QComboBox()
        self.cmb_conn_type.addItem('TCP/IP', 'tcp')
        self.cmb_conn_type.addItem('COM-порт', 'serial')
        self.cmb_conn_type.currentTextChanged.connect(self.on_conn_type_changed)
        
        lay_conn.addWidget(self.lbl_conn_type, 0, 0)
        lay_conn.addWidget(self.cmb_conn_type, 0, 1)
        
        # Поля для TCP-подключения
        self.lbl_host = QLabel('IP-адрес:')
        self.edit_host = QLineEdit(self.tcp_host)
        self.lbl_port = QLabel('Порт:')
        self.edit_port = QLineEdit(str(self.tcp_port))
        
        lay_conn.addWidget(self.lbl_host, 1, 0)
        lay_conn.addWidget(self.edit_host, 1, 1)
        lay_conn.addWidget(self.lbl_port, 2, 0)
        lay_conn.addWidget(self.edit_port, 2, 1)
        
        # Поля для COM-подключения
        self.lbl_serial_port = QLabel('COM-порт:')
        self.cmb_serial_port = QComboBox()
        self.refresh_serial_ports()
        self.btn_refresh_ports = QPushButton('Обновить')
        self.btn_refresh_ports.clicked.connect(self.refresh_serial_ports)
        self.lbl_baud = QLabel('Скорость (baud):')
        self.cmb_baud = QComboBox()
        for baud in [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]:
            self.cmb_baud.addItem(str(baud), baud)
        self.cmb_baud.setCurrentText(str(self.serial_baud))
        
        lay_conn.addWidget(self.lbl_serial_port, 3, 0)
        lay_conn.addWidget(self.cmb_serial_port, 3, 1)
        lay_conn.addWidget(self.btn_refresh_ports, 3, 2)
        lay_conn.addWidget(self.lbl_baud, 4, 0)
        lay_conn.addWidget(self.cmb_baud, 4, 1)
        
        # Кнопка подключения и статус
        self.lbl_status = QLabel('Статус: Не подключен')
        self.lbl_status.setStyleSheet('QLabel {color:red}')
        self.btn_conn = QPushButton('Подключиться')
        self.btn_conn.clicked.connect(self.toggle_connection)
        lay_conn.addWidget(self.lbl_status, 5, 0)
        lay_conn.addWidget(self.btn_conn, 5, 1)
        group_conn.setLayout(lay_conn)
        main.addWidget(group_conn)

        # ---------- 2. Питание ----------
        group_power = QGroupBox('Управление питанием')
        lay_power = QHBoxLayout()
        self.btn_on  = QPushButton('Включить лазер')
        self.btn_off = QPushButton('Выключить лазер')
        self.btn_qp  = QPushButton('Запросить статус')
        self.btn_on.clicked.connect(self.power_on)
        self.btn_off.clicked.connect(self.power_off)
        self.btn_qp.clicked.connect(self.query_power)
        lay_power.addWidget(self.btn_on)
        lay_power.addWidget(self.btn_off)
        lay_power.addWidget(self.btn_qp)
        group_power.setLayout(lay_power)
        main.addWidget(group_power)

        # ---------- 3. Яркость ----------
        group_br = QGroupBox('Управление яркостью')
        lay_br = QVBoxLayout()

        # текущая яркость + запрос
        cur_br = QHBoxLayout()
        self.lbl_cur_br = QLabel('Текущая яркость: неизвестно')
        self.btn_qb = QPushButton('Запросить яркость')
        self.btn_qb.clicked.connect(self.query_brightness)
        cur_br.addWidget(self.lbl_cur_br)
        cur_br.addWidget(self.btn_qb)

        # ползунок
        slider_br = QHBoxLayout()
        self.sld_br = QSlider(Qt.Horizontal)
        self.sld_br.setRange(0, 255)
        self.sld_br.setValue(128)
        self.lbl_br_val = QLabel(str(self.sld_br.value()))
        self.sld_br.valueChanged.connect(
            lambda v: self.lbl_br_val.setText(str(v))
        )
        slider_br.addWidget(self.sld_br)
        slider_br.addWidget(self.lbl_br_val)

        # кнопка «установить»
        set_br = QHBoxLayout()
        self.btn_set_br = QPushButton('Установить яркость')
        self.btn_set_br.clicked.connect(self.set_brightness)
        set_br.addWidget(self.btn_set_br)

        lay_br.addLayout(cur_br)
        lay_br.addLayout(slider_br)
        lay_br.addLayout(set_br)
        group_br.setLayout(lay_br)
        main.addWidget(group_br)

        # ---------- 4. Углы ----------
        group_angle = QGroupBox('Управление углом освещения')
        grid = QGridLayout()

        # spot angle
        self.lbl_spot = QLabel('Угол светового пятна (градусы):')
        self.sb_spot_deg = QSpinBox()  # отображаемый спинбокс в градусах
        self.sb_spot_deg.setRange(1, 71)  # по спецификации от 1.8 до 71 градуса
        self.sb_spot_deg.setValue(2)  # начальное значение около 1.8 градуса
        self.sb_spot_deg.valueChanged.connect(self.spot_deg_value_changed)
        
        # скрытый спинбокс для внутреннего представления (0-0x4000)
        self.sb_spot = QSpinBox()
        self.sb_spot.setRange(0, 0x4000)
        self.sb_spot.setValue(4000)  # начальное значение для 1.8 градуса
        self.sb_spot.hide()  # скрываем этот спинбокс
        self.sb_spot.valueChanged.connect(self.spot_internal_value_changed)
        
        # связываем внутреннее значение с отображаемым
        self.spot_deg_value_changed(2)  # инициализация
        
        self.btn_set_spot = QPushButton('Установить угол')
        self.btn_set_spot.clicked.connect(self.set_spot_angle)

        # motor angle
        self.lbl_motor = QLabel('Позиция мотора (градусы):')
        self.sb_motor_deg = QSpinBox()  # отображаемый спинбокс в градусах
        self.sb_motor_deg.setRange(-180, 180)  # стандартный диапазон для углов
        self.sb_motor_deg.setValue(0)  # начальное значение
        self.sb_motor_deg.valueChanged.connect(self.motor_deg_value_changed)
        
        # скрытый спинбокс для внутреннего представления (0-0x4000)
        self.sb_motor = QSpinBox()
        self.sb_motor.setRange(0, 0x4000)
        self.sb_motor.setValue(8192)  # начальное значение
        self.sb_motor.hide()  # скрываем этот спинбокс
        self.sb_motor.valueChanged.connect(self.motor_internal_value_changed)
        
        # связываем внутреннее значение с отображаемым
        self.motor_deg_value_changed(0)  # инициализация
        
        self.btn_set_motor = QPushButton('Установить позицию')
        self.btn_set_motor.clicked.connect(self.set_motor_angle)

        # запросы
        self.btn_q_spot  = QPushButton('Запросить угол света')
        self.btn_q_motor = QPushButton('Запросить позицию мотора')
        self.btn_q_spot.clicked.connect(self.query_spot_angle)
        self.btn_q_motor.clicked.connect(self.query_motor_angle)

        # увеличение/уменьшение
        self.btn_inc = QPushButton('Увеличить угол')
        self.btn_dec = QPushButton('Уменьшить угол')
        self.btn_inc.clicked.connect(self.increase_angle)
        self.btn_dec.clicked.connect(self.decrease_angle)

        # сброс
        self.btn_reset = QPushButton('Сброс мотора')
        self.btn_reset.clicked.connect(self.reset_motor)

        # размещаем в сетке
        grid.addWidget(self.lbl_spot,    0, 0)
        grid.addWidget(self.sb_spot_deg, 0, 1)  # показываем градусный спинбокс
        grid.addWidget(self.btn_set_spot,0, 2)

        grid.addWidget(self.lbl_motor,   1, 0)
        grid.addWidget(self.sb_motor_deg, 1, 1)  # показываем градусный спинбокс
        grid.addWidget(self.btn_set_motor,1, 2)

        grid.addWidget(self.btn_q_spot,  2, 0)
        grid.addWidget(self.btn_q_motor, 2, 1)

        grid.addWidget(self.btn_inc,     3, 0)
        grid.addWidget(self.btn_dec,     3, 1)
        grid.addWidget(self.btn_reset,   3, 2)

        group_angle.setLayout(grid)
        main.addWidget(group_angle)

        # ---------- 5. Доп. функции ----------
        group_misc = QGroupBox('Дополнительные функции')
        grid_misc = QGridLayout()

        # fan
        self.btn_fan_q = QPushButton('Запросить статус вентилятора')
        self.btn_fan_q.clicked.connect(self.query_fan)

        # version
        self.btn_ver_q = QPushButton('Запросить версию ПО')
        self.btn_ver_q.clicked.connect(self.query_version)

        # address
        self.cmb_addr = QComboBox()
        for i in range(1, 255):
            self.cmb_addr.addItem(str(i))
        self.cmb_addr.setCurrentText('1')
        self.btn_addr_set = QPushButton('Изменить адрес')
        self.btn_addr_set.clicked.connect(self.change_address)

        # baud rate (для устройства, а не соединения)
        self.cmb_dev_baud = QComboBox()
        self.baud_map = {
            0x00: 1200,   0x01: 2400,   0x02: 4800,
            0x03: 9600,   0x04: 19200,  0x05: 38400,
            0x06: 57600,  0x07: 115200
        }
        for code, rate in self.baud_map.items():
            self.cmb_dev_baud.addItem(str(rate), code)
        self.cmb_dev_baud.setCurrentIndex(3)               # 9600 по умолчанию
        self.btn_baud_set = QPushButton('Изменить скорость')
        self.btn_baud_set.clicked.connect(self.change_baud)

        grid_misc.addWidget(self.btn_fan_q,      0, 0)
        grid_misc.addWidget(self.btn_ver_q,      0, 1)
        grid_misc.addWidget(QLabel('Адрес:'),    1, 0)
        grid_misc.addWidget(self.cmb_addr,       1, 1)
        grid_misc.addWidget(self.btn_addr_set,   1, 2)
        grid_misc.addWidget(QLabel('Скорость:'), 2, 0)
        grid_misc.addWidget(self.cmb_dev_baud,   2, 1)
        grid_misc.addWidget(self.btn_baud_set,   2, 2)

        group_misc.setLayout(grid_misc)
        main.addWidget(group_misc)

        # ---------- 6. Журнал ----------
        group_log = QGroupBox('Журнал обмена данными')
        vlog = QVBoxLayout()
        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        vlog.addWidget(self.log_edit)
        group_log.setLayout(vlog)
        main.addWidget(group_log)

        self.setLayout(main)
        self.update_connection_fields()  # скрыть/показать поля в зависимости от типа подключения

    # ------------------------------------------------------------------
    def spot_deg_value_changed(self, value):
        """Обновляем внутреннее значение при изменении отображаемого значения в градусах"""
        # Преобразуем градусы в внутреннее значение (0-0x4000)
        # Спецификация: дальний угол 1.8°, ближний угол 71°
        # Примем, что 1.8° = 4000 (максимальное значение для spot), 71° = 0
        if value >= 1.8:
            # масштабируем: 1.8° = 4000, 71° = 0
            # линейное преобразование: внутр_знач = (71 - градусы) / (71 - 1.8) * 4000
            internal_value = round((71 - value) / (71 - 1.8) * 0x4000)
            # ограничиваем в допустимом диапазоне
            internal_value = max(0, min(0x4000, internal_value))
            self.sb_spot.setValue(internal_value)

    def spot_internal_value_changed(self, value):
        """Обновляем отображаемое значение при изменении внутреннего значения"""
        # Преобразуем внутреннее значение обратно в градусы
        # Внутр_знач = 0 соответствует 71°, внутр_знач = 4000 соответствует 1.8°
        degrees = 71 - (value / 0x4000 * (71 - 1.8))
        degrees = max(1.8, min(71, degrees))
        self.sb_spot_deg.setValue(int(round(degrees)))  # Используем int() для округления до целого

    def motor_deg_value_changed(self, value):
        """Обновляем внутреннее значение при изменении отображаемого значения в градусах"""
        # Преобразуем градусы в внутреннее значение (0-0x4000)
        # Масштабируем от -180° до 180° в диапазон 0-0x4000
        # Формула: внутр_знач = ((градус + 180) / 360) * 0x4000
        internal_value = round(((value + 180) / 360) * 0x4000)
        # ограничиваем в допустимом диапазоне
        internal_value = max(0, min(0x4000, internal_value))
        self.sb_motor.setValue(internal_value)

    def motor_internal_value_changed(self, value):
        """Обновляем отображаемое значение при изменении внутреннего значения"""
        # Преобразуем внутреннее значение обратно в градусы
        # Внутр_знач = 0 соответствует -180°, внутр_знач = 0x4000 соответствует 180°
        degrees = (value / 0x4000 * 360) - 180
        self.sb_motor_deg.setValue(int(round(degrees)))
    # ------------------------------------------------------------------
    # ------------------- Пакет‑утилиты -------------------------------
    # ------------------------------------------------------------------
    def _calc_checksum(self, data_without_checksum: list) -> int:
        """Сумма всех байт (address…data2) & 0xFF."""
        return sum(data_without_checksum) & 0xFF

    def build_packet(self, cmd1: int, cmd2: int,
                     data1: int = 0, data2: int = 0) -> bytes:
        """Создаёт пакет: FF | address | cmd1 | cmd2 | data1 | data2 | checksum."""
        pkt = [0xFF, self.device_address, cmd1, cmd2, data1, data2]
        pkt.append(self._calc_checksum(pkt[1:]))   # checksum от address до data2
        return bytes(pkt)

    def send_packet(self, packet: bytes, timeout: float = 5.0):
        """Запускает отдельный поток, сохраняет ссылку, подключает сигналы."""
        if not self.connection:
            QMessageBox.warning(self, 'Нет подключения',
                                'Сначала подключитесь к лазеру!')
            return

        worker = Worker(self.connection, packet, timeout, conn_type=self.conn_type)
        # Сохраняем, иначе GC может удалить объект до окончания работы
        self._workers.append(worker)

        # По завершении удаляем из списка, чтобы не накапливались ссылки
        worker.finished.connect(lambda w=worker: self._workers.remove(w))

        worker.responseReceived.connect(self._on_response)
        worker.errorOccurred.connect(self._on_error)
        worker.start()

    # ------------------------------------------------------------------
    # -------------------- Слоты от Worker ---------------------------
    # ------------------------------------------------------------------
    @pyqtSlot(bytes, bytes)
    def _on_response(self, response: bytes, request: bytes):
        """Обрабатываем полученный ответ (может быть пустой)."""
        if not response:                     # «тихая» команда, ответ не нужен
            self.log_message(
                f'Команда отправлена, ответ не требуется: {self._hex(request)}'
            )
            return

        self.log_message(f'Ответ: {self._hex(response)}')
        self.process_response(response, request)

    @pyqtSlot(str)
    def _on_error(self, msg: str):
        self.log_message(f'Ошибка соединения: {msg}')
        QMessageBox.critical(self, 'Ошибка соединения', msg)

    # ------------------------------------------------------------------
    # ------------------------- Лог ---------------------------------
    # ------------------------------------------------------------------
    def _hex(self, data: bytes) -> str:
        return ' '.join(f'0x{b:02X}' for b in data)

    def log_message(self, text: str):
        """Записывает строку в журнал, оставляя максимум 500 строк."""
        self.log_edit.append(text)
        if self.log_edit.document().blockCount() > 500:
            cursor = self.log_edit.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    # ------------------------------------------------------------------
    # ------------------- Работа с соединением ------------------------
    # ------------------------------------------------------------------
    def on_conn_type_changed(self):
        self.conn_type = self.cmb_conn_type.currentData()
        self.update_connection_fields()

    def update_connection_fields(self):
        # Показать/скрыть поля в зависимости от типа подключения
        is_tcp = self.conn_type == 'tcp'
        is_serial = self.conn_type == 'serial'
        
        # TCP поля
        self.lbl_host.setVisible(is_tcp)
        self.edit_host.setVisible(is_tcp)
        self.lbl_port.setVisible(is_tcp)
        self.edit_port.setVisible(is_tcp)
        
        # Serial поля
        self.lbl_serial_port.setVisible(is_serial)
        self.cmb_serial_port.setVisible(is_serial)
        self.btn_refresh_ports.setVisible(is_serial)
        self.lbl_baud.setVisible(is_serial)
        self.cmb_baud.setVisible(is_serial)

    def refresh_serial_ports(self):
        self.cmb_serial_port.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        for port in ports:
            self.cmb_serial_port.addItem(port)
        if ports:
            self.cmb_serial_port.setCurrentIndex(0)

    def toggle_connection(self):
        if self.connection is None:
            if self.connect_to_laser():
                self.lbl_status.setText('Статус: Подключён')
                self.lbl_status.setStyleSheet('QLabel {color:green}')
                self.btn_conn.setText('Отключиться')
        else:
            self.disconnect_from_laser()
            self.lbl_status.setText('Статус: Не подключен')
            self.lbl_status.setStyleSheet('QLabel {color:red}')
            self.btn_conn.setText('Подключиться')

    def connect_to_laser(self) -> bool:
        try:
            if self.conn_type == 'tcp':
                # Получаем параметры TCP-подключения из интерфейса
                self.tcp_host = self.edit_host.text()
                self.tcp_port = int(self.edit_port.text())
                
                self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.connection.settimeout(10)
                self.connection.connect((self.tcp_host, self.tcp_port))
                self.log_message(f'Подключено к {self.tcp_host}:{self.tcp_port} по TCP')
            else:  # serial
                # Получаем параметры COM-подключения из интерфейса
                self.serial_port = self.cmb_serial_port.currentText()
                self.serial_baud = int(self.cmb_baud.currentText())
                
                self.connection = serial.Serial(self.serial_port, self.serial_baud, timeout=10)
                self.log_message(f'Подключено к {self.serial_port} на скорости {self.serial_baud} по COM-порту')
            return True
        except Exception as exc:
            QMessageBox.critical(
                self, 'Ошибка подключения',
                f'Не удалось подключиться через {self.conn_type.upper()}.\n{exc}'
            )
            self.connection = None
            return False

    def disconnect_from_laser(self):
        if self.connection:
            try:
                if self.conn_type == 'tcp':
                    self.connection.close()
                else:  # serial
                    self.connection.close()
                self.log_message('Соединение закрыто')
            finally:
                self.connection = None

    # ------------------------------------------------------------------
    # --------------------- Обработчики UI‑элементов -----------------
    # ------------------------------------------------------------------
    # Питание
    def power_on(self):
        pkt = self.build_packet(0x01, 0x01, 0x01, 0x00)
        self.send_packet(pkt, timeout=2)

    def power_off(self):
        pkt = self.build_packet(0x01, 0x01, 0x00, 0x00)
        self.send_packet(pkt, timeout=2)

    def query_power(self):
        pkt = self.build_packet(0x02, 0x01, 0x00, 0x00)
        self.send_packet(pkt, timeout=5)

    # Яркость
    def set_brightness(self):
        val = self.sld_br.value()
        pkt = self.build_packet(0x01, 0x03, val, 0x00)
        self.send_packet(pkt, timeout=2)

    def query_brightness(self):
        pkt = self.build_packet(0x02, 0x03, 0x00, 0x00)
        self.send_packet(pkt, timeout=5)

    # Углы
    def set_spot_angle(self):
        # используем внутреннее значение для отправки команды
        ang = self.sb_spot.value()
        hi, lo = (ang >> 8) & 0xFF, ang & 0xFF
        pkt = self.build_packet(0x08, 0x01, hi, lo)
        self.send_packet(pkt, timeout=5)

    def set_motor_angle(self):
        # используем внутреннее значение для отправки команды
        ang = self.sb_motor.value()
        hi, lo = (ang >> 8) & 0xFF, ang & 0xFF
        pkt = self.build_packet(0x01, 0x05, hi, lo)
        self.send_packet(pkt, timeout=5)

    def query_spot_angle(self):
        pkt = self.build_packet(0x09, 0x01, 0x00, 0x00)
        self.send_packet(pkt, timeout=5)

    def query_motor_angle(self):
        pkt = self.build_packet(0x02, 0x05, 0x00, 0x00)
        self.send_packet(pkt, timeout=5)

    def increase_angle(self):
        pkt = self.build_packet(0x01, 0x04, 0x01, 0x00)
        self.send_packet(pkt, timeout=2)

    def decrease_angle(self):
        pkt = self.build_packet(0x01, 0x04, 0x00, 0x00)
        self.send_packet(pkt, timeout=2)

    def reset_motor(self):
        pkt = self.build_packet(0x01, 0x06, 0x00, 0x00)
        self.send_packet(pkt, timeout=15)

    # Вентилятор, версия
    def query_fan(self):
        pkt = self.build_packet(0x02, 0x0F, 0x00, 0x00)
        self.send_packet(pkt, timeout=5)

    def query_version(self):
        pkt = self.build_packet(0x05, 0x10, 0x01, 0x01)
        self.send_packet(pkt, timeout=5)

    # Адрес
    def change_address(self):
        new_addr = int(self.cmb_addr.currentText())
        pkt = self.build_packet(0x03, 0x11, 0x00, new_addr)
        self.send_packet(pkt, timeout=5)

    # Скорость
    def change_baud(self):
        code = self.cmb_dev_baud.currentData()          # уже хранит 0x00‑0x07
        pkt = self.build_packet(0x03, 0x13, 0x00, code)
        self.send_packet(pkt, timeout=5)

    # ------------------------------------------------------------------
    # --------------------- Обработка ответов ------------------------
    # ------------------------------------------------------------------
    def process_response(self, resp: bytes, request: bytes):
        """
        По протоколу каждый пакет – ровно 7 байт:
        0: FF (header)
        1: address
        2: cmd1
        3: cmd2
        4: data1
        5: data2
        6: checksum
        """
        if len(resp) != 7:
            self.log_message('Ответ имеет неверный размер')
            return

        hdr, addr, cmd1, cmd2, d1, d2, chk = resp

        if hdr != 0xFF:
            self.log_message('Некорректный заголовок в ответе')
            return
        if addr != self.device_address:
            self.log_message(f'Ответ от другого адреса: {addr:#02x}')
            return
        if self._calc_checksum(list(resp[1:6])) != chk:
            self.log_message('Контрольная сумма ответа не совпадает')
            return

        # --------------------------------------------------------------
        # 1) Ответы на запросы
        # --------------------------------------------------------------
        if (cmd1, cmd2) == (0x02, 0x01):               # статус питания
            state = d1
            if state == 0x00:
                self.lbl_status.setText('Статус: Подключён (Лазер ВЫКЛ)')
                self.lbl_status.setStyleSheet('QLabel {color:orange}')
                self.log_message('Лазер выключен')
            elif state == 0x01:
                self.lbl_status.setText('Статус: Подключён (Лазер ВКЛ)')
                self.lbl_status.setStyleSheet('QLabel {color:green}')
                self.log_message('Лазер включен')
            else:
                self.log_message(f'Неизвестный статус питания: {state:#02x}')

        elif (cmd1, cmd2) == (0x02, 0x03):               # яркость
            br = d1
            self.lbl_cur_br.setText(f'Текущая яркость: {br}')
            self.log_message(f'Текущая яркость = {br}')

        elif (cmd1, cmd2) == (0x02, 0x05):               # позиция мотора
            ang = (d1 << 8) | d2
            # Преобразуем внутреннее значение в градусы для отображения
            degrees = (ang / 0x4000 * 360) - 180
            self.log_message(f'Позиция мотора = {degrees:.1f}° ({ang} внутр. ед.)')
            # Обновляем отображаемое значение в интерфейсе
            self.sb_motor_deg.setValue(int(round(degrees)))
            self.sb_motor.setValue(ang)

        elif (cmd1, cmd2) == (0x02, 0x0F):               # статус вентилятора
            fan = d1
            self.log_message(f'Вентилятор: {"ВКЛ" if fan == 0x01 else "ВЫКЛ"}')

        elif (cmd1, cmd2) == (0x09, 0x01):               # угол света
            ang = (d1 << 8) | d2
            # Преобразуем внутреннее значение в градусы для отображения
            degrees = 71 - (ang / 0x4000 * (71 - 1.8))
            degrees = max(1.8, min(71, degrees))
            self.log_message(f'Угол светового пятна = {degrees:.1f}° ({ang} внутр. ед.)')
            # Обновляем отображаемое значение в интерфейсе
            self.sb_spot_deg.setValue(int(round(degrees)))  # Используем int() для округления до целого
            self.sb_spot.setValue(ang)

        elif (cmd1, cmd2) == (0x20, 0x00):               # версия (major/minor)
            # Протокол: pp=major, qq=minor, tt=patch — но пакет 7 байт, tt не передаётся
            major, minor = d1, d2
            self.log_message(f'Версия ПО: {major}.{minor}')

        elif (cmd1, cmd2) == (0x21, 0x00):               # дата версии
            # Протокол: pp=год, qq=месяц, tt=день — но пакет 7 байт, день не передаётся
            year  = 2000 + d1
            month = d2
            self.log_message(f'Дата сборки: {year}-{month:02d}')

        elif (cmd1, cmd2) == (0x22, 0x00):               # тип модуля
            typ = (d1 << 8) | d2
            self.log_message(f'Тип модуля: 0x{typ:04X}')

        # --------------------------------------------------------------
        # 2) Неизвестный тип ответа
        # --------------------------------------------------------------
        else:
            self.log_message(f'Необработанный ответ: {self._hex(resp)}')


# ----------------------------------------------------------------------
#   Точка входа
# ----------------------------------------------------------------------
def main() -> None:
    app = QApplication(sys.argv)
    w = LaserController()
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()