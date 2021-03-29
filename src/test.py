#!/usr/bin/env python
# -*- coding: utf-8 -*-

from protocol import SerialProxy
import threading
import sys
import time 
from cam import Camera

import rospy
from std_msgs.msg import String

        # p.subscribe_for_opcode(0x44, lambda x: p.send_packet(0x99, 0x23))

from std_msgs.msg import String

# rospy.Subscriber("/y", String, lambda x: print("@#@", x))

if len(sys.argv) > 1:
    p = None
    try:
        p = SerialProxy("/dev/pts/8", 115200, topic_postfix='/dev')
        # p.subscribe_for_opcode(0x44, lambda x: p.send_packet(0x99, 0x23))
        # p._subscribe_request('/test'.encode())
        p.subscribe('/test')
        # p.spin()
        p.request_image(1, 10)
        while True:
            pass
    except KeyboardInterrupt:
        p.close()
    finally:
        p.close()
else:
    p = None
    # p = rospy.Publisher(topic, AnyMsg, queue_size=10)

    try:
        p = SerialProxy("/dev/pts/7", 115200, topic_postfix='/dev', camera_controller=Camera())
        # p.subscribe_for_opcode(0x99, lambda x: print("!!!!", x))
        # p.su(0x44, bytes([0x11]))
        # p.spin()
        
        while True:
            pass
            # p.spin()
            # time.sleep(1)
    except KeyboardInterrupt:
        p.close()
    finally:
        p.close()
