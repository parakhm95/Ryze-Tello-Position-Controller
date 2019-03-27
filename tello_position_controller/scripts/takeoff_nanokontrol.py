#!/usr/bin/env python

# This is nanokontrol2 controlling the drone

import rospy
import math

import sys
import time
from sensor_msgs.msg import Joy
import subprocess
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String
from std_srvs.srv import SetBool

import roslib

roslib.load_manifest('std_srvs')
from std_srvs.srv import Trigger
import std_srvs.srv
from mav_manager.srv import Vec4
from mav_manager.srv import GoalTimedRequest
from mav_manager.srv import GoalTimed
from tello_position_controller.msg import GoTo

gotoservice1, gotoservice2 = None, None
quad1 = None
quad2 = None
# goal1 = GoalTimedRequest()
goal2 = GoTo()
initial_quad1_z = None
initial_quad2_z = None
initial_quad2_y = None
initial_quad1_y = None
initial_quad1_w = None
initial_quad1_x = None
initial_quad2_x = None
joyz = 0.00
startup = 0
joya = 0.00


def quad_agg_1(pose_msg):
    global quad1, initial_quad1_z, initial_quad1_x
    # print("quad_agg_1")
    quad1 = pose_msg
    if initial_quad1_z is None:
        initial_quad1_z = quad1.pose.position.z
        initial_quad1_x = quad1.pose.position.x
        # print(quad1)


def quad_agg_2(pose_msg):
    global quad2, initial_quad2_z, initial_quad2_y, initial_quad2_x
    quad2 = pose_msg
    if initial_quad2_z is None:
        initial_quad2_z = quad2.pose.position.z
        initial_quad2_y = quad2.pose.position.y
        initial_quad2_x = quad2.pose.position.x
    # publ()


def publ():
    if startup is 1:
        # x1 = quad1.pose.position.x
        # y1 = quad1.pose.position.y
        # z1 = quad1.pose.position.z
        x2 = quad2.pose.position.x
        y2 = quad2.pose.position.y
        z2 = quad2.pose.position.z
        # u = (x1 - x2, y1 - y2, z1 - z2)
        # theta = math.asin((z1 - z2) / math.sqrt(u[0] ** 2 + u[1] ** 2 + u[2] ** 2))

        teemer = rospy.get_time()
        # goal1.goal=([quad2.pose.position.x+0.200,quad2.pose.position.y,quad1.pose.position.z+(1+joy_msg.axes[0])/2,0.0])
        # goal2.goal=([quad2.pose.position.x,quad2.pose.position.y,quad2.pose.position.z+(1+joy_msg.axes[0])/2,0.0])
        # struct_x = ((x1 + x2)*0.5 + ((initial_quad1_x+initial_quad2_x)/2)*0.)
        # struct_y = initial_quad2_y
        # struct_y = ((y1 + y2)*0.4 + (initial_quad2_y)*0.2)
        # struct_x = 0.4 * x1 + 0.4 * x2 + 0.2 *(initial_quad1_x +  initial_quad2_x)/2. 
        # struct_z = (((quad1.pose.position.z+quad2.pose.position.z)*1.4)+(0.2*(quad1_z+joyz)))/3
        struct_z = initial_quad2_z + joyz
        # goal1.goal = ([struct_x - 0.100, struct_y, struct_z  + 0 * 0.220 * math.sin(0.8 * theta), 0.0])
        goal2.goal = ([initial_quad2_x + joya, initial_quad2_y, struct_z, 0.0])
        # goal1.duration.secs = 0
        # goal1.t_start.secs = teemer + 0.1
        gotoservice2(goal2)
        # gotoservice1(goal1)
        # print(goal1.goal[1])
        # print(theta)
        # print(quad2)


def callback(joy_msg):
    rospy.wait_for_service('/tello/takeoff')
    global joyz, startup, joya, goal2
    toff = rospy.ServiceProxy('/tello/takeoff', Trigger)
    land = rospy.ServiceProxy('/tello/land', Trigger)
    emergency = rospy.ServiceProxy('/tello/emergency', Trigger)

    if quad2 is None:
        rospy.logwarn('No Odometry')
        return

    print("nanokontrol change")

    if joy_msg.buttons[26] == 1:
        # toff1()
        toff()
        print("Taking-off now")
        startup = 1

    if joy_msg.buttons[29] == 1:
        startup = 0
        emergency()
        print("Motors off emergency")

    if joy_msg.buttons[28] == 1:
        land()
        print("Going to ground")

    if joy_msg.buttons[24] == 1:
        joyz = (1 + joy_msg.axes[0])
        print("publish trajectory")

    if joy_msg.buttons[24] == 1:
        joya = (1 + joy_msg.axes[1])
        print("publish trajectory")

    if joy_msg.buttons[16] == 1:
        startup = 1
        x1 = quad1.pose.position.x
        y1 = quad1.pose.position.y
        z1 = quad1.pose.position.z
        w1 = 2*math.acos(quad1.pose.orientation.w)
        goal2.x,goal2.y,goal2.z,goal2.yaw = x1-0.072*math.sin(w1),y1 + 0.058*math.cos(w1),z1+0.4,w1

    if joy_msg.buttons[17] == 1:
        startup = 1
        x1 = quad1.pose.position.x
        y1 = quad1.pose.position.y
        z1 = quad1.pose.position.z
        w1 = 2*math.acos(quad1.pose.orientation.w)
        goal2.x,goal2.y,goal2.z,goal2.yaw = x1-0.072*math.sin(w1),y1 + 0.058*math.cos(w1),z1+0.20,w1

    if joy_msg.buttons[18] == 1:
        startup = 1
        x1 = quad1.pose.position.x
        y1 = quad1.pose.position.y
        z1 = quad1.pose.position.z
        w1 = 2*math.acos(quad1.pose.orientation.w)
        goal2.x,goal2.y,goal2.z,goal2.yaw = x1-0.072*math.sin(w1),y1 + 0.058*math.cos(w1),z1+0.7,w1




def listener():
    global startup
    rospy.init_node('takeoff_nanokontrol', anonymous=True)
    rospy.Subscriber("/vicon/octahold/pose", PoseStamped, quad_agg_1)
    rospy.Subscriber("/vicon/tello/pose", PoseStamped, quad_agg_2)
    rospy.Subscriber("/nanokontrol2", Joy, callback)
    goal_pub=rospy.Publisher('/tello/GoTo', GoTo, queue_size=0)
    freq=100  # hz
    rate=rospy.Rate(freq)
    while not rospy.is_shutdown():
        if startup is 1:
            goal_pub.publish(goal2)
            startup = 0
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
