#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import sys
from time import sleep

#from opcodes import Opcodes
#from protocol import SerialProxy
#from utils import to_2b
import math
import os

import numpy as np
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QLabel, QLineEdit, QPushButton, QComboBox, QListWidget, QVBoxLayout
from PyQt5.QtCore import Qt, QRect

from protocol import SerialProxy


class ImageScroller(QtWidgets.QWidget):
    def __init__(self):
        self.idealSize = 1
        self.idealSizePoints = []
        self.points = []
        self.idealSizeCheck = False
        self.roverSize = 20
        self.chosen_points = []
        self.points = []
        self.idealPoints = []
        self.sm = 0
        self.clearCheck = False

        self.p = SerialProxy("/dev/ttyUSB1", 115200)

        QtWidgets.QWidget.__init__(self)
        self._image = QtGui.QPixmap("D:/IT-квантум_Кванториум63/qtNTI/main.png")

        self.coords = QLabel(self)
        self.coords.setText("Координаты: None, None")
        self.coords.setFixedSize(400, 50)

        self.coords.move(30, 30)


        self.combo = QComboBox(self)
        self.combo.addItems(["INTERNAL","WAIT", "DRIVE", "DRIVE_TARGET", "TURN_TARGET", "TURN_TILL", "CIRCLE", "ANTI_COLLIDE", "AUTO_LEAVE",
                             "CAM_PITCH", "CAM_YAW", "CAM_REQ", "CAM_YAW_ADD",
                             "MANIPULATOR_GRIP", "MANIPULATOR_ARROW", "MANIPULATOR_YAW", "MANIPULATOR_EXEC",
                             "GET_DIST", "GET_ODOM", "ANY"])


        self.combo.move(30, 90)

        self.qle = QLineEdit(self)
        self.qle.move(150, 90)

        sendBtn = QPushButton(self)
        sendBtn.move(150, 170)
        sendBtn.setText('Send')
        sendBtn.clicked.connect(self.SendBtn)

        clearBtn = QPushButton(self)
        clearBtn.move(450, 570)
        clearBtn.setText('Clear')
        clearBtn.clicked.connect(self.clearBtn)

        vbox = QVBoxLayout(self)

        self.listWidget = QListWidget()

        self.listWidget.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.listWidget.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.listWidget.setFixedSize(190, 250)

        vbox.addWidget(self.listWidget)
        self.setLayout(vbox)

        sendPackBtn = QPushButton(self)
        sendPackBtn.move(150, 510)
        sendPackBtn.setText('SendPack')
        sendPackBtn.clicked.connect(self.sendPackBtn)


        self.setGeometry(0, 0, 1200, 700)
        self.setWindowTitle('aaaa')
        self.show()

    def clearBtn(self):
        self.idealSize = 1
        self.idealSizePoints = []
        self.points = []
        self.idealSizeCheck = False
        self.chosen_points = []
        self.points = []
        self.idealPoints = []
        self.sm = 0
        print("\n" * 100)
        self.clearCheck = True


    def Drive(self, distance, convert=True):
        if convert:
            print(f'проехать {distance / self.sm} сантиметров прямо')
        else:
            print(f'проехать {distance} сантиметров прямо')

    def to_2b(self, data):
        return data // 256, data % 256

    def SendBtn(self):
        if len(self.qle.text()) > 0:
            data = int(self.qle.text()) #текст из поля для ввода
            opcodeText = self.combo.currentText() #выбранный элемент в списке
            textToHex = {"INTERNAL": "0x00", "WAIT": "0x07",
                         "DRIVE": "0xA0", "DRIVE_TARGET": "0xA1", "TURN_TARGET": "0xA2", "TURN_TILL": "0xA3", "CIRCLE": "0xA4", "ANTI_COLLIDE": "0xA5", "AUTO_LEAVE": '0xA6',
                         "CAM_PITCH": "0xB1", "CAM_YAW": "0xB2", "CAM_REQ": "0xB0", "CAM_YAW_ADD": '0xB3',
                         "MANIPULATOR_GRIP": "0xC0", "MANIPULATOR_ARROW": "0xC1", "MANIPULATOR_YAW": "0xC2",
                         "MANIPULATOR_EXEC": "0xC3",
                         "GET_DIST": "0xD0", "GET_ODOM": "0xD1", "ANY": "0xFF"}


            #self.p.append_cmd(eval(f'Opcodes.{opcodeText}'), self.to_2b(data))
            self.listWidget.addItem(f"{str(textToHex[opcodeText])} {str(self.to_2b(data))}")
            print(textToHex[opcodeText], self.to_2b(data))

    def sendPackBtn(self):

        self.listWidget.clear()
        self.sendPackToRover()

    def sendPackToRover(self):
        pass
       # self.p.send_cmd_packet()


    def setAngle(self, deg):
        print(f'повернуться на {180 - deg} градусов')

    def paintEvent(self, paint_event):
        painter = QtGui.QPainter(self)
        painter.drawPixmap(QRect(300, 20, self._image.width(), self._image.height()), self._image)
        pen = QtGui.QPen()
        pen.setWidth(4)

        if self.clearCheck:
            self.clearCheck = False
            self.update()

        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        for pos in self.chosen_points:
            pen.setColor(QColor(255, 0, 0))
            painter.setPen(pen)
            painter.drawPoint(pos)

        for pos in self.idealPoints:
            pen.setColor(QColor(0, 255, 0))
            painter.setPen(pen)
            painter.drawPoint(pos)

    def mouseReleaseEvent(self, cursor_event):
        if self._image.width() + 300 > cursor_event.x() > 300 and self._image.height() + 20 > cursor_event.y() > 20:

            self.coords.setText(f"Координаты:{cursor_event.x()}, {cursor_event.y()}")

            if (cursor_event.button() == Qt.LeftButton):
                if self.idealSizeCheck:
                    self.chosen_points.append(cursor_event.pos())
                    self.points.append([cursor_event.x(), cursor_event.y()])
            elif (cursor_event.button() == Qt.RightButton):
                if len(self.points) > 0:
                    for i in range(1, len(self.points)):
                        if i < len(self.points) - 1:

                            m = math.hypot(self.points[i][0] - self.points[i - 1][0],
                                           self.points[i][1] - self.points[i - 1][1])

                            A = (self.points[i - 1][0], self.points[i - 1][1])
                            B = (self.points[i][0], self.points[i][1])
                            C = (self.points[i + 1][0], self.points[i + 1][1])

                            a = np.radians(np.array(A))
                            b = np.radians(np.array(B))
                            c = np.radians(np.array(C))
                            avec = a - b
                            cvec = c - b
                            lat = b[0]
                            avec[1] *= math.cos(lat)
                            cvec[1] *= math.cos(lat)
                            angle2deg = np.degrees(
                                math.acos(np.dot(avec, cvec) / (np.linalg.norm(avec) * np.linalg.norm(cvec))))

                            self.Drive(m)
                            self.setAngle(angle2deg)

                        else:
                            m = math.hypot(self.points[i][0] - self.points[i - 1][0],
                                           self.points[i][1] - self.points[i - 1][1])

                            self.Drive(m)

            elif (cursor_event.button() == Qt.MidButton):
                if self.idealSize <= 2:
                    self.idealSize += 1
                    self.idealPoints.append(cursor_event.pos())
                    self.idealSizePoints.append([cursor_event.x(), cursor_event.y()])
                else:
                    self.idealSizeCheck = True
                    a = math.hypot(self.idealSizePoints[1][0] - self.idealSizePoints[0][0],
                                   self.idealSizePoints[1][1] - self.idealSizePoints[0][1])
                    self.sm = round(a / self.roverSize, 3)
            self.update()



if __name__ == '__main__':
    import sys

    app = QtWidgets.QApplication(sys.argv)
    w = ImageScroller()
    w.show()
    sys.exit(app.exec_())



