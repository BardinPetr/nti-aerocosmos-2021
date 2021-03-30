from math import sqrt

from nti_acs.msg import CamPos, Manipulator

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16
from tf.transformations import euler_from_quaternion


class Hardware:
    BASE_SPD = 1

    imu = None
    odom = None
    sonar = 0

    def __init__(self, spd_map=1, spd_map_angular=1):
        self.cam_pos = rospy.Publisher("/ard/cam_pos", CamPos, queue_size=0)
        self.manipulator = rospy.Publisher("/ard/manipulator", Manipulator, queue_size=0)
        self.vel = rospy.Publisher("/cmd_vel", Twist, queue_size=0)

        self.sub_imu = rospy.Subscriber("/imu", Imu, lambda x: self.update_imu(x))
        self.sub_odom = rospy.Subscriber("/odom", Odometry, lambda x: self.update_odom(x))
        self.sub_dist = rospy.Subscriber("/ard/sonar", UInt16, lambda x: self.update_sonar(x))

        self.global_spd_coef = spd_map
        self.global_angular_spd_coef = spd_map_angular

    def update_imu(self, x):
        self.imu = x

    def update_odom(self, x):
        self.odom = x

    def update_sonar(self, x):
        self.sonar = x

    def location(self):
        return self.odom.pose.pose.position

    def orientation(self):
        return euler_from_quaternion(self.odom.pose.pose.orientation)

    @staticmethod
    def dist(a, b):
        return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def drive(self, spd_x, spd_y, spd_coef=None):
        if spd_coef is None:
            spd_coef = self.global_spd_coef
        self.vel.publish(Twist(spd_x * spd_coef, spd_y * spd_coef, 0, 0, 0, 0))

    def stop(self):
        self.drive(0, 0)

    def turn(self, spd):
        self.vel.publish(Twist(0, 0, 0, 0, 0, spd * self.global_angular_spd_coef))

    def drive_dist(self, dst):
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
        start = self.orientation()
        while abs((self.orientation()[2] - start[2]) % 360) < angle:
            pass
        self.stop()

    def circle(self, r, mode):
        pass

    def set_cam(self, pitch, yaw):
        self.cam_pos.publish(pitch, yaw)

    def set_manipulator(self, arrow0, arrow1, yaw, grip):
        self.manipulator.publish(arrow0, arrow1, yaw, grip)

    def place_beacon(self, i):
        pass  # TODO
