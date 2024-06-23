#!/usr/bin/env python
#coding:utf-8
#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php
#

import sys, rospy, math, tf
from pimouse_ros.msg  import MotorFreqs
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry

class Motor():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)

        rospy.on_shutdown(self.set_power)
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.callback_scan)        
        self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
        self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
        self.using_cmd_vel = False
        self.srv_tm = rospy.Service('timed_motion', TimedMotion, self.callback_tm)

        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

        scan_no_max = 726 #for simpleURG
        self.scan = [0]*(scan_no_max*2) #① Which measurement angle+② Double the number of elements to save in the scan data set
        self.scan_no = 0
        self.angle_min = -2.35619449615 #for simpleURG -135deg
        self.angle_max = 2.09234976768 #for simpleURG 120deg
        self.angle_increment = 0.00613592332229 #for simpleURG 0.36deg
        self.angle_total_n = scan_no_max
        self.scan_time_secs = 0
        self.scan_time_nsecs = 0        

        path1 = '/home/ubuntu/output_data/URG.dat'
        self.f = open(path1,'w')

    def set_power(self, onoff=False):
        en = "/dev/rtmotoren0"
        try:
            with open(en,'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False

    def set_raw_freq(self, left_hz, right_hz):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return

        try:
            with open("/dev/rtmotor_raw_l0",'w') as lf,\
                 open("/dev/rtmotor_raw_r0",'w') as rf:
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            rospy.logerr("cannot write to rtmotor_raw_*")

    def onoff_response(self, onoff):
        d = TriggerResponse()
        d.success = self.set_power(onoff)
        d.message = "ON" if self.is_on else "OFF"
        return d

    def send_odom(self):
        self.cur_time = rospy.Time.now()
        dt = self.cur_time.to_sec() - self.last_time.to_sec()
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt
        if self.th > 3.14:
            self.th = self.th - 6.28
        elif self.th < -3.14:
            self.th = self.th + 6.28

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x, self.y, 0.0), q, self.cur_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        if self.scan_no > 0 : #Make the same format as the input data file of SLAM in this book
            output_list =["LASERSCAN"]
            output_list.append(self.scan_no)
            output_list.append(self.cur_time.secs)
            output_list.append(self.cur_time.nsecs)
            output_list.append('340') #(723-44)/2 Save scan data from the sensor in half.
            for i in range (44, 724, 1): #Roughly saves scan data from -120deg to 120deg
                output_list.append(self.scan[i])
            output_list.append(self.x) #Odometry X direction
            output_list.append(self.y) #Odometry Y direction
            output_list.append(self.th) #Save with Odometry rotation angle RAD
            output_list.append(self.scan_time_secs)
            output_list.append(self.scan_time_nsecs)
            for d in output_list:
                self.f.write("%s " %d)
            self.f.write("\n")
                 
        self.pub_odom.publish(odom)
        self.last_time = self.cur_time

    def callback_raw_freq(self,message):
        self.set_raw_freq(message.left_hz,message.right_hz)

    def callback_cmd_vel(self,message):
        if not self.is_on:
            return
        self.vx = message.linear.x
        self.vth = message.angular.z

        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)
        
        self.using_cmd_vel = True

    def callback_scan(self,message):
        self.scan_no = message.header.seq
        self.scan_time_secs = message.header.stamp.secs
        self.scan_time_nsecs = message.header.stamp.nsecs
        for i in range (0, self.angle_total_n * 2, 4):
            #Measurement angle index (DEG)
            self.scan[int(i/2)] = (self.angle_min + (i/2) * self.angle_increment) * 180 / 3.14 
            if math.isnan(message.ranges[int(i/2)]):
                #When the measurement data is nan (outside the measurement range), 0 is assigned
                self.scan[int(i/2)+1] = 0 
            else:
                self.scan[int(i/2)+1] = message.ranges[int(i/2)] #scan data

    def callback_on(self, message):
        return self.onoff_response(True)

    def callback_off(self, message):
        return self.onoff_response(False)

    def callback_tm(self, message):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return False

        dev = "/dev/rtmotor0"
        try:
            with open(dev,'w') as f:
                f.write("%d %d %d\n" %(message.left_hz, message.right_hz, message.duration_ms))

        except:
            rospy.logerr("cannot write to " + dev)
            return False

        return True

if __name__=='__main__':
    rospy.init_node('motors')
    m = Motor()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom()
        rate.sleep()
    m.f.close()
