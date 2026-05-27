#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Тестовый скрипт для проверки стилей и новой системы дебаунса приложения Laser Controller
"""

import sys
from PyQt5.QtWidgets import QApplication

# Импортируем главный класс из основного файла
from laser import LaserController

def main():
    app = QApplication(sys.argv)
    
    # Создаем главное окно
    window = LaserController()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()