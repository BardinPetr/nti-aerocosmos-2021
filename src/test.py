#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

from cam import Camera
from protocol import SerialProxy
import signal

# p.subscribe_for_opcode(0x44, lambda x: p.send_packet(0x99, 0x23))

p = None

def die(n, f):
    p.close()
    exit(0)


signal.signal(signal.SIGINT, die)
signal.signal(signal.SIGTERM, die)


if len(sys.argv) > 1:
    p = SerialProxy("/dev/pts/10", 115200, topic_postfix='/dev')
    # p.begin_cmd_packet()
    # p.append_cmd(0x09, [0x34, 0x76])
    # p.append_cmd(0x47, [0x43, 0x69])
    # p.send_cmd_packet()
    p.request_image(1, 10)
    while True:
        pass
else:
    p = SerialProxy("/dev/pts/11", 115200, is_robot=True, topic_postfix='/dev', camera_controller=Camera())
    # p.subscribe_for_opcode(0x09, print)
    # p.subscribe_for_opcode(0x47, print)

    while True:
        pass
