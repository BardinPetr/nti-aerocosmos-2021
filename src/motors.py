from math import sqrt

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from src.nti_acs.src.utils import from_2b
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Driver:
    BASE_SPD = 1

    imu = None
    odom = None

    def __init__(self):
        self.vel = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        self.sub_imu = rospy.Subscriber("/imu", Imu, lambda x: self.update_imu(x))
        self.sub_odom = rospy.Subscriber("/odom", Odometry, lambda x: self.update_odom(x))

    def update_imu(self, _imu):
        self.imu = _imu

    def update_odom(self, _odom):
        self.odom = _odom

    def location(self):
        return self.odom.pose.pose.position

    def orientation(self):
        return euler_from_quaternion(self.odom.pose.pose.orientation)

    def dist(self, a, b):
        return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def drive(self, spd_x, spd_y):
        self.vel.publish(Twist(spd_x, spd_y, 0, 0, 0, 0))

    def stop(self):
        self.drive(0, 0)

    def turn(self, spd):
        self.vel.publish(Twist(0, 0, 0, spd, 0, 0))

    def drive_dist(self, raw):
        if self.odom is None:
            raise SystemError("Odom is not published yet")
        dst = from_2b(*raw)
        self.drive(self.BASE_SPD, 0)
        start = self.location()
        while self.dist(start, self.location()) < dst:
            pass
        self.stop()

    def turn_angle(self, raw):
        if self.odom is None:
            raise SystemError("Odom is not published yet")
        angle = from_2b(*raw)
        self.turn(self.BASE_SPD)
        start = self.orientation()
        while abs((self.orientation()[2] - start[2]) % 360) < angle:
            pass
        self.stop()
