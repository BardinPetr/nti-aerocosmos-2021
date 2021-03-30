#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal

from protocol import SerialProxy, Opcodes
from src.nti_acs.src.utils import to_2b

p = SerialProxy("/dev/pts/10", 115200, topic_postfix='/dev')


def die(n, f):
    p.close()
    exit(0)


signal.signal(signal.SIGINT, die)
signal.signal(signal.SIGTERM, die)

p.begin_cmd_packet()
p.append_cmd(0xA1, to_2b(10))
p.append_cmd(Opcodes.WAIT, to_2b(3000))
p.append_cmd(0xA1, to_2b(0))
p.send_cmd_packet()

p.request_image(1, 10)

while True:
    pass
