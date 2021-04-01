#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from threading import Thread

from future import standard_library

standard_library.install_aliases()

from builtins import object
from builtins import int

from time import sleep
import cv2


class Camera(object):
    def __init__(self, cameras=None, size=(400, 300)):
        self.reader_thread = {}
        self.camera = {}
        self.size = size
        if cameras is None:
            i = 0
            while True:
                self.camera[i] = cv2.VideoCapture(i)
                if self.camera[i].read()[0]:
                    self.reader_thread[i] = Thread(target=lambda: self.update_cam(i))
                    self.reader_thread[i].daemon = True
                    self.reader_thread[i].start()
                else:
                    del self.camera[i]
                    if i > 20:
                        break
                i += 1
        else:
            self.camera = {i: cv2.VideoCapture(i) for i in cameras}
        print("Cams used:", self.camera.keys())

    def update_cam(self, i):
        while True:
            _ = self.camera[i].read()
            sleep(1 / 1000)

    def prepare(self, image, quality=10):
        if image is str:
            image = cv2.imread(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.resize(image, self.size)
        res = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), quality,
             int(cv2.IMWRITE_JPEG_OPTIMIZE), 1]
        )[1]
        return res.reshape(-1).tobytes()

    def get(self, i, quality=10, retry=10):
        if len(self.camera) == 0:
            return None
        if i not in self.camera:
            i = list(self.camera.keys())[(10 - retry) % len(self.camera)]
        try:
            ret, img = self.camera[i].read()
            if ret:
                return self.prepare(img)
        except:
            pass
        if retry > 0:
            sleep(0.05)
            return self.get(i, quality, retry - 1)
        return None

    def close(self):
        for j, i in self.camera.items():
            i.release()
            self.reader_thread[j].terminate()

# c = Camera()
#
# print("Q")
# open('a.jpg', 'wb').write(c.get(1, 10))
# sleep(3)
# print("Q")
# open('b.jpg', 'wb').write(c.get(1, 10))
