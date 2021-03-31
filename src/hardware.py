from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import time
from time import sleep

from future import standard_library
standard_library.install_aliases()

from builtins import bytes
from builtins import object
from math import sqrt, radians

from nti_acs.msg import CamPos, ManipulatorArrow

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16, Int16
from tf.transformations import euler_from_quaternion


class Hardware(object):
    GRIPPER_OPENED = 80
    GRIPPER_CLOSED = 90

    BASE_SPD = 0.5

    imu = None
    odom = None
    sonar = 0

    def __init__(self, spd_map=1, spd_map_angular=1):
        self.cam_pos = rospy.Publisher("/ard/cam_pos", CamPos, queue_size=10, latch=True)
        self.vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)

        self.man_arrow = rospy.Publisher("/ard/manip_arrow", ManipulatorArrow, queue_size=10, latch=True)
        self.man_grip = rospy.Publisher("/ard/manip_grip", Int16, queue_size=10, latch=True)
        self.man_yaw = rospy.Publisher("/ard/manip_yaw", Int16, queue_size=10, latch=True)

        self.ard_status = rospy.Publisher("/ard/status", Int16, queue_size=10, latch=True)

        self.sub_imu = rospy.Subscriber("/imu", Imu, lambda x: self.update_imu(x))
        self.sub_odom = rospy.Subscriber("/odom", Odometry, lambda x: self.update_odom(x), )
        self.sub_dist = rospy.Subscriber("/ard/sonar", UInt16, lambda x: self.update_sonar(x))

        self.global_spd_coef = spd_map
        self.global_angular_spd_coef = spd_map_angular

    def update_imu(self, x):
        self.imu = x

    def update_odom(self, x):
        self.odom = x
        # print(x)

    def update_sonar(self, x):
        self.sonar = x

    def location(self):
        return self.odom.pose.pose.position

    def orientation(self):
        quaternion = self.odom.pose.pose.orientation
        return euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

    @staticmethod
    def dist(a, b):
        return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def drive(self, spd_x, spd_y, spd_coef=None):
        if spd_coef is None:
            spd_coef = self.global_spd_coef
        t = Twist()
        t.linear.x = spd_x * spd_coef
        t.linear.y = spd_y * spd_coef
        self.vel.publish(t)

    def stop(self):
        self.drive(0, 0)

    def turn(self, spd):
        t = Twist()
        t.angular.z = spd * self.global_angular_spd_coef
        self.vel.publish(t)

    def drive_dist(self, dst_cm):
        dst = dst_cm / 100
        if self.odom is None:
            raise SystemError("Odom is not published yet")
        self.drive(self.BASE_SPD, 0)
        start = self.location()
        while self.dist(start, self.location()) < dst:
            pass
        self.stop()

    def turn_angle(self, angle):
        if self.odom is None:
            raise SystemError("Odom is not published yet")
        self.turn(self.BASE_SPD)
        start = self.orientation()[2]
        angle = radians(angle)
        while abs(self.orientation()[2] - start) < angle:
            # print(abs(self.orientation()[2] - start))
            sleep(0.005)
        self.stop()

    def circle(self, r, mode):
        pass

    def set_cam(self, pitch, yaw):
        self.cam_pos.publish(pitch, yaw)

    def set_gripper(self, grip=None, angle=None):
        if grip is not None:
            self.set_gripper(angle=(self.GRIPPER_CLOSED if grip else self.GRIPPER_OPENED))
        elif angle is not None:
            self.man_grip.publish(angle)

    def set_manipulator_arrow(self, a0, a1):
        self.man_arrow.publish(a0, a1)

    def set_manipulator_yaw(self, a):
        self.man_yaw.publish(a)

    def place_beacon(self, i):
        pass  # TODO


# rospy.init_node("main", anonymous=True)
# h = Hardware()

# sleep(1)

# h.turn_angle(180)