#!/usr/bin/env python
# coding: utf-8

import sys, rospy, math, tf
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class GetInfo():
    def __init__(self):
        rospy.on_shutdown(self.myhook)
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.callback_scan)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.callback_odom)

        scan_no_max = 360 #for LDS01(turtlebot3)
        self.scan = [0]*(scan_no_max*2)
        self.scan_no = 0
        self.angle_min = 0. #for LSD01(turtlebot3)
        self.angle_max = 6.26573181152 #for LDS01(turtlebot3)
        self.angle_increment = 0.0174532923847 #for LDS01(turtlebot3)
        self.angle_total_n = scan_no_max
        self.scan_time_secs = 0
        self.scan_time_nsecs = 0

        self.odom_x = 0.
        self.odom_y = 0.
        self.odom_th = 0.
        self.odom_time_secs = 0
        self.odom_time_nsecs = 0
        
        path_w = '/home/shuro/output_data/LDS01.dat' #自分の環境に応じて保存先を変更する
        self.f = open(path_w,'w')

    def callback_scan(self,message):
        self.scan_no = message.header.seq
        self.scan_time_secs = message.header.stamp.secs
        self.scan_time_nsecs = message.header.stamp.nsecs
        for i in range (0, self.angle_total_n*2, 2):
            self.scan[i]=(self.angle_min+(i/2)*self.angle_increment)*180/3.14
            laser_data = message.ranges[int(i/2)]
            if math.isnan(laser_data):
                self.scan[i+1] = 0
            else:
                self.scan[i+1] = laser_data
        
    def callback_odom(self,message):
        self.odom_time_secs = message.header.stamp.secs
        self.odom_time_nsecs = message.header.stamp.nsecs
        self.odom_x = message.pose.pose.position.x
        self.odom_y = message.pose.pose.position.y

        quaternion = (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.odom_th = euler[2]
        #print("%f %f" %(self.odom_x, self.odom_y))

    def output(self):

        if self.scan_no > 0 :
            output_list =["LASERSCAN"]
            output_list.append(self.scan_no)
            output_list.append(self.scan_time_secs)
            output_list.append(self.scan_time_nsecs)
            output_list.append('360') #取得データは全周(360deg)に対して1degずつのデータ
            for i in range (0, 360*2, 1):
                output_list.append(self.scan[i])
            output_list.append(self.odom_x)
            output_list.append(self.odom_y)
            output_list.append(self.odom_th)
            output_list.append(self.odom_time_secs)
            output_list.append(self.odom_time_nsecs)
            for d in output_list:
                self.f.write("%s " %d)
            self.f.write("\n")

    def myhook(self):
        print("shut down")

if __name__=='__main__':
    rospy.init_node('get_info')
    g = GetInfo()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        g.output()
        rate.sleep()
    g.f.close()
