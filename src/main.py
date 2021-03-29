import signal

from protocol import SerialProxy
from src.nti_acs.src.motors import Driver

p = SerialProxy("/dev/pts/10", 115200, topic_postfix='/dev')


def die(n, f):
    p.close()
    exit(0)


signal.signal(signal.SIGINT, die)
signal.signal(signal.SIGTERM, die)

d = Driver()

p.subscribe_for_opcode(0xA1, d.drive_dist)
p.subscribe_for_opcode(0xA2, d.turn_angle)
