#!/usr/bin/env python
# coding=utf-8

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
# from std_msgs import Float64  Float64MultiArray
from std_msgs.msg import Float64
import time
import math
import numpy as np

# keyboard input
import threading
import Tkinter


class Px4Controller:

    def __init__(self):
        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.local_enu_position = None

        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"
        self.imu_rate = np.array([0, 0, 0])

        self.state = None
        self.mavros_state = State()
        self.command = TwistStamped()
        self.mav_pos = np.array([0, 0, 0])
        self.mav_vel = np.array([0, 0, 0])
        self.cmd_vel = np.array([0, 0, 0])
        self.cmd_body_vel = np.array([0, 0, 0])
        self.cmd_force_vel = np.array([0, 0, 0])
        self.ttc = 0
        self.cmd_pos_vel = np.array([0, 0, 0])
        # self.q = Queue()
        self.mav_yaw = 0
        self.cmd_yaw = 0
        self.mav_roll = 0
        self.mav_pitch = 0
        self.mav_R = np.zeros((3, 3))
        self.desire_pos = np.array([200, -15, 2.5])  # [20, 30, 0]
        self.start_yaw = 0

        '''
        ros subscribers   /balance_force  /ttc_mean
        '''
        self.local_pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.gps_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber(
            "/mavros/imu/data", Imu, self.imu_callback)
        self.balance_force = rospy.Subscriber(
            "/balance_force", Float64, self.force_callback)
        self.ttc_mean = rospy.Subscriber(
            "/ttc_mean", Float64, self.ttc_callback)
        '''
        ros publishers
        '''
        self.vel_pub = rospy.Publisher(
            'mavros/flow_vel', TwistStamped, queue_size=10)

        print("Px4 lll Controller Initialized!")

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg
        # self.mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.mav_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.mav_yaw = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        self.mav_pitch = math.asin(2*(q0*q2 - q1*q3))
        self.mav_roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
        R_ae = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                         [2*(q1*q2+q0*q3), q0**2-q1**2 +
                             q2**2-q3**2, 2*(q2*q3-q0*q1)],
                         [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
        R_ba = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        self.mav_R = R_ae.dot(R_ba)  # 机体坐标系  x————向右  y————向前   z————向上
        print("mav is ok!!")

    def force_callback(self, msg):
        self.cmd_force_vel = np.array([0, 0, 0])
        self.cmd_force_vel[0] = 0.07 * msg.data  # 0.1
        # print("cmd_force: {}".format(self.cmd_force_vel[0]))
        self.cmd_force_vel = self.mav_R.dot(self.cmd_force_vel)  # body to NEU
        self.cmd_force_vel[2] = 0

        self.command.twist.linear.x = self.cmd_force_vel[0]
        self.command.twist.linear.y = self.cmd_force_vel[1]
        self.command.twist.linear.z = 0
        self.command.twist.angular.z = 0
        self.vel_pub.publish(self.command)
        print("cmd_force is ok!!")
        print("connected: {}".format(self.cmd_force_vel))
        # self.cmd_force_vel[2] = 0

    def start(self):
        rospy.init_node("offboard_node")

        rospy.spin()

    def ttc_callback(self, msg):
        self.ttc = msg.data   #

    def mav_vel_cb(self, msg):
        self.mav_vel = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
    # mav_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

    def SatIntegral(self, a, up, down):
        for i in range(len(a)):
            if a[i] > up:
                a[i] = up
            elif a[i] < down:
                a[i] = down
        return a

    def Saturation(self, a, up, down):
        if a > up:
            a = up
        elif a < down:
            a = down
        return a

    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg
        self.imu_rate = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.current_heading = self.q2yaw(self.imu.orientation)
        self.received_imu = True

    def gps_callback(self, msg):
        self.gps = msg

    def q2yaw(self, q):
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))


if __name__ == '__main__':
    con = Px4Controller()
    con.start()
