#!/usr/bin/env python
#coding:utf-8
#keyboard_cmd_vel.py
#Copyright (c) 2016 RT Corp. <shop@rt-net.jp>
#Copyright (c) 2016 Daisuke Sato <tiryoh@gmail.com>
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>

#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

rospy.wait_for_service('/motor_on')
rospy.wait_for_service('/motor_off')
rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
rospy.ServiceProxy('/motor_on',Trigger).call()

rospy.init_node('keyboard_cmd_vel')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
while not rospy.is_shutdown():
    vel = Twist()
    direction = raw_input('i: forward, ,: backward, j: left forward, l: left backward, return: stop > ')    
    if 'i' in direction: vel.linear.x = 0.20
    if ',' in direction: vel.linear.x = -0.20
    if 'j' in direction: vel.linear.x = 0.10; vel.angular.z = 1.57
    if 'l' in direction: vel.linear.x = -0.10; vel.angular.z = -1.57
    print vel
    pub.publish(vel)
