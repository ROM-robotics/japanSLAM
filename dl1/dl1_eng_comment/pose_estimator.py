#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import math

from pose2d import Pose2D
from scan2d import Scan2D
from pose_optimizer import PoseOptimizer
from data_associator import DataAssociator


class PoseEstimatorICP:
    def __init__(self, curScan=None, usedNum=0, pnrate=0., popt=None, dass=None):
        self.curScan = curScan if curScan else Scan2D()
        self.usedNum = usedNum
        self.pnrate = pnrate
        self.popt = popt if popt else PoseOptimizer()
        self.dass = dass if dass else DataAssociator()

    def getUsedNum(self):
        return self.usedNum

    def setScanPair_scan2d_GT(self, c, r):
        self.curScan = c
        self.dass.setRefBaseGT(r.lps)  # Register reference scan points for data correspondence

    # Give the initial value INITPOSE and find the estimated Estpose for the robot position by ICP    
    def estimatePose(self, initPose, estPose):
        evmin = math.inf  # Cost minimum. The initial value is large
        evthre = 0.000001 #Cost change threshold. If the change amount is less than this, repeat.The larger the value of this value, the better the result
        self.popt.setEvthre(evthre*0.1) #Actually, the set value is trial and error
        self.popt.setEvlimit(0.2)  # evlimitIs the threshold of the outdated value [M]
        ev = 0.  # cost
        evold = evmin  # One previous value. Used for convergence judgment
        pose = initPose
        poseMin = initPose
        for i in range(100):  # i<100Is a measure against vibration, but if the number of repetitions, such as I = 200, may increase the quality.There is a trial and error part.
            if i > 0:
                evold = ev
            mratio, pose = self.dass.findCorrespondenceGT(self.curScan, pose)  # Data support
            newPose = Pose2D()
            self.popt.setPoints(self.dass.curLps, self.dass.refLps)  # Give the response results
            ev, newPose = self.popt.optimizePoseSL(pose, newPose)  # Optimization of robot position in response
            pose = newPose
            if ev < evmin:  # Save cost minimum results
                poseMin = newPose
                evmin = ev
            if evold > ev and evold - ev < evthre:
                break
            elif ev > evold  and ev - evold < evthre:
                break
        self.pnrate = self.popt.getPnrate()
        self.usedNum = len(self.dass.curLps)
        estPose = poseMin
        return evmin, estPose
