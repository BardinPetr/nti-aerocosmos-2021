from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import pickle

from future import standard_library

standard_library.install_aliases()
from pyblake2 import blake2b
from io import BytesIO

import roslib
import rostopic

import sys
import threading


class KThread(threading.Thread):
    """A subclass of threading.Thread, with a kill()
  method."""

    def __init__(self, *args, **keywords):
        threading.Thread.__init__(self, *args, **keywords)
        self.killed = False

    def start(self):
        """Start the thread."""
        self.__run_backup = self.run
        self.run = self.__run
        threading.Thread.start(self)

    def __run(self):
        """Hacked run function, which installs the
    trace."""
        sys.settrace(self.globaltrace)
        self.__run_backup()
        self.run = self.__run_backup

    def globaltrace(self, frame, why, arg):
        if why == 'call':
            return self.localtrace
        else:
            return None

    def localtrace(self, frame, why, arg):
        if self.killed:
            if why == 'line':
                raise SystemExit()
        return self.localtrace

    def kill(self):
        self.killed = True


def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)


def drive_to_robot(x, y):
    x = 127 if x == 0 else int(translate(x, -1, 1, 0, 255))
    y = 127 if y == 0 else int(translate(y, -1, 1, 0, 255))
    return x, y


def drive_on_robot(x, y):
    x = 0 if x == 127 else translate(x, 0, 255, -1, 1)
    y = 0 if y == 127 else translate(y, 0, 255, -1, 1)
    return x, y


def encode_topic(data):
    return blake2b(
        data.encode('utf-8') if type(data) is str else data,
        digest_size=2
    ).digest()


def msg_object_by_topic(topic):
    return roslib.message.get_message_class(rostopic.get_topic_type('/test')[0])


def preprocess_types(data):
    if type(data) is list:
        res = bytearray()
        for i in data:
            res.extend(preprocess_types(i))
    elif 'serialize' in dir(data):
        s = BytesIO()
        data.serialize(s)
        res = s.getvalue()
    elif type(data) is str:
        res = data.encode('utf-8')
    elif type(data) is int:
        res = [data]
    else:
        res = data
    return res


def to_2b(data):
    return data // 256, data % 256


def from_2b(a, b):
    return 256 * a + b


def postprocess_odom(raw):
    return pickle.loads(raw)


def preprocess_odom(hardware):
    loc = hardware.location()
    return pickle.dumps([
        loc.x,
        loc.y,
        hardware.orientation()[2]]
    )
