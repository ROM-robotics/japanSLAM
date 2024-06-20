#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import numpy as np
import math

from my_util import MyUtil
from pose2d import Pose2D
from scan2d import Scan2D
from pose_graph import PoseGraph
from point_cloud_map import PointCloudMap
from cost_function import CostFunction
from data_associator import DataAssociator
from pose_estimator import PoseEstimatorICP
from pose_fuser import PoseFuser
from covariance_calculator import CovarianceCalculator

# Loop arc setting information
class LoopInfo:
    def __init__(self, arcked=False, curId=-1, refId=-1, pose=None, score=-1., cov=None):
        self.arcked = arcked  # Have you already put a pose arc?
        self.curId = curId  # Currently key frame ID (scan)
        self.refId = refId  # Reference key frame ID (scan or localgridmap2d)
        # Currently a global posture where the key frame matches the reference key frame (reverse in the case of Grid base)
        self.pose = pose if pose else Pose2D()
        self.score = score  # ICP matching score
        self.cov = cov if cov else np.zeros((3, 3))  # Scattered

    def setArcked(self, t):
        self.arcked = t

class LoopDetector:
    def __init__(self, pg=None, radius=1., atdthre=5., scthre=0.2):
        self.pg = pg if pg else PoseGraph()  # Pose graph
        self.radius = radius  # Exploration radius [M] (distance threshold for the current location and revisited)
        self.atdthre = atdthre  # Cumulative mileage difference threshold [M]
        self.atdthre2 = 1.  # Skills of mileage after LoopDetec [M]
        self.prevDetectionPose = Pose2D(math.inf, math.inf)  # Pose for LoopDetec just before
        self.scthre = scthre  # ICP score threshold
        self.pcmap = PointCloudMap()  # Point group map
        self.cfunc = CostFunction()  # Cost function (used separately from ICP)
        self.estim = PoseEstimatorICP()  # Robot position estimate (ICP)
        self.dass = DataAssociator()  # Data -compatible device
        self.pfu = PoseFuser()  # Sensor fusioner

    def setPoseGraph(self, p):
        self.pg = p

    def setPoseEstimator(self, p):
        self.estim = p

    def setPoseFuser(self, p):
        self.pfu = p

    def setDataAssociator(self, d):
        self.dass = d

    def setPointCloudMap(self, p):
        self.pcmap = p

    # Loop inspection
    # Close to the current location CURPOSE, find a place where the shape matches the scan CURSCAN from the robot trajectory and put a pose arc
    def detectLoop(self, curScan, curPose, cnt):
        print("-- detectLoop -- ")
        # Find the closest part map
        atd = self.pcmap.atd  # Current actual cumulative mileage
        atdR = 0  # Cumulative mileage when tracing the trajectory with the following processing
        poses = self.pcmap.poses  # Robot trajectory
        dmin = math.inf  # The minimum distance of the distance to the previous visit point
        imin = 0
        jmin = 0  # Index of the lowest distance last time
        prevP = Pose2D()  # Robot position just before
        len_self_pcmap_submaps_1 = len(self.pcmap.submaps) - 1
        math_sqrt = math.sqrt
        atdFromPrev = (curPose.tx - self.prevDetectionPose.tx) * (curPose.tx - self.prevDetectionPose.tx) + (curPose.ty - self.prevDetectionPose.ty) * (curPose.ty - self.prevDetectionPose.ty)
        if atdFromPrev < self.atdthre2:  # When the mileage after the loop is short, the loop is not detected
            print("Already Loop Detected: dis=%f, (x,y)=%f %f"%(atdFromPrev,self.prevDetectionPose.tx,self.prevDetectionPose.ty))
            return False
        for i in range(0, len_self_pcmap_submaps_1, 1):  # Find something other than the current part map
            submap = self.pcmap.submaps[i]  # I, the II partial map
            for j in range(submap.cntS, submap.cntE, 1):  # About each robot position on the part of the part
                p = poses[j]  # Robot position
                atdR += math_sqrt((p.tx - prevP.tx) * (p.tx - prevP.tx) + (p.ty - prevP.ty) * (p.ty - prevP.ty))
                if atd - atdR < self.atdthre:  # If the mileage to the current location is short, do not consider it as a loop, stop
                    i = len(self.pcmap.submaps)  # Now you can get out of the outer loop
                    break
                prevP = p
                d = (curPose.tx - p.tx) * (curPose.tx - p.tx) + (curPose.ty - p.ty) * (curPose.ty - p.ty)
                if d < dmin:  # Is the distance between the current location and P minimum so far
                    dmin = d
                    imin = i  # Index of partial maps that are candidates
                    jmin = j  # Last time index
        print("dmin=%f, radius=%f, imin=%d, jmin=%d" % (math.sqrt(dmin), self.radius, imin, jmin))  # For confirmation
        if dmin > self.radius * self.radius:  # If the distance to the last visit point is long, the loop will not be detected
            return False
        refSubmap = self.pcmap.submaps[imin]  # See the closest part map to scan
        initPose = poses[jmin]

        # Finding the location of the revisit
        revisitPose = Pose2D()
        flag, revisitPose = self.estimateRevisitPose(curScan, refSubmap.mps, curPose, revisitPose)

        if flag:  # Detected the loop
            icpCov = np.empty([3, 3])  # ICP coordination
            icpCov = self.pfu.calIcpCovariance(revisitPose, curScan, icpCov)  # Calculate ICP coordination
            info = LoopInfo()  # Loop detection result
            info.pose = revisitPose  # Set a revisited position for loop arc information
            info.cov = icpCov  # Set coordinates for loop arc information.
            info.curId = cnt  # Current position node ID
            info.refId = int(jmin)  # Node ID of the previous visit point
            self.makeLoopArc(info)  # Loop arc generation
            self.prevDetectionPose = revisitPose #Once detected, do not detect between Atdthre2.

        return flag

    # The previous visiting point (Refid) is the start -out node, and the current location (CURID) is the terminal node to generate a loop arc.
    def makeLoopArc(self, info):
        if info.arcked:  # INFO arc is already stretched
            return
        info.setArcked(True)
        srcPose = self.pcmap.poses[info.refId]  # Position of the previous visit point
        dstPose = Pose2D(info.pose.tx, info.pose.ty, info.pose.th)  # Location position
        relPose = Pose2D()
        relPose = dstPose.calRelativePose(srcPose, relPose)  # Loop Arc restraint
        # Since the restraint of Ark is a relative position from the starting point node, converting coordination to the starting point node coordinate system of the loop arc
        cov = np.empty([3, 3])
        cov = CovarianceCalculator.rotateCovariance(srcPose, info.cov, cov, True)  # Reverse rotation of coordinates
        arc = self.pg.makeArc(info.refId, info.curId, relPose, cov)  # Loop arc generation
        self.pg.addArc(arc)  # Loop arc registration

    # Currently, ICP is performed with scan Curscan and partial map point group Reflps, and the position of a revisiting point is obtained.
    def estimateRevisitPose(self, curScan, refLps, initPose, revisitPose):
        self.dass.setRefBaseGT(refLps)  # Set the reference group in the data compatible device
        self.cfunc.setEvlimit(0.2)  # Cost function error threshold
        print("initPose: tx=%f, ty=%f, th=%f" % (initPose.tx, initPose.ty, initPose.th))  # For confirmation
        usedNumMin = 50  # 100
        # Initial position Initpose Investigate the area around the INITPOSE
        # ICP is not performed to improve efficiency, Simply check the matching score at each position
        rangeT = 0.5 #org 1. # Parallel search range[m]
        rangeA = 25. #org 45.  # Rotation search range [degree]
        dd = 0.2  # Parallel search interval[m]
        da = 2.  # Exploration interval of rotation [degree]
        scoreMin = 1000.
        scores = np.empty(0)
        candidates = np.empty(0)  # Good score candidate position
        for dy in np.arange(-rangeT, rangeT + dd, dd):  # Repeated exploration of parallel Y
            y = initPose.ty + dy  # Add DY to the initial position
            for dx in np.arange(-rangeT, rangeT + dd, dd):  # Repeated exploration of parallel X
                x = initPose.tx + dx  # Add DX to the initial position
                for dth in np.arange(-rangeA, rangeA + da, da):  # Repeat rotation search
                    th = MyUtil.add(initPose.th, dth)  # Add DTH to the initial position
                    pose = Pose2D(x, y, th)
                    mratio, pose = self.dass.findCorrespondenceGT(curScan, pose)  # Data correspondence with position Pose
                    usedNum = len(self.dass.curLps)
                    if usedNum < usedNumMin or mratio < 0.9:  # Fly if the response rate is bad
                        continue
                    self.cfunc.setPoints(self.dass.curLps, self.dass.refLps)  # Set a point group for cost functions
                    score = self.cfunc.calValuePD(x, y, th)  # Cost value (matching score)
                    pnrate = self.cfunc.getPnrate()  # Detailed response rate
                    if pnrate > 0.8:
                        candidates = np.append(candidates, pose)
                        if score < scoreMin:
                            scoreMin = score
                        scores = np.append(scores, score)
        len_candidates = len(candidates)
        print("candidates.size=%d" % len_candidates)  # For confirmation
        if len_candidates == 0:
            flag = 0
            return flag, revisitPose

        # Select the best one of the candidate position Candidates by ICP
        best = Pose2D()  # Best candidate
        smin = 1000000.  # ICP score smaller
        self.estim.setScanPair_l_point2d_GT(curScan, refLps)  # Scan settings for ICP
        for i in range(len_candidates):
            p = candidates[i]  # Alternate position
            print("candidates %d (%d)" % (i, len_candidates))  # For confirmation
            estP = Pose2D()
            score, estP = self.estim.estimatePose(p, estP)  # Find the matching position with ICP
            pnrate = self.estim.getPnrate()  # Rate rate of points in ICP
            usedNum = self.estim.getUsedNum()  # Points used in ICP
            print("score=%f, pnrate=%f, usedNum=%d" % (score, pnrate, usedNum))  # For confirmation
            if score < smin and pnrate >= 0.9 and usedNum >= usedNumMin:  # Loop detection is severe
                smin = score
                best = estP

        # I found it if the minimum score was smaller than the threshold
        if smin <= self.scthre:
            revisitPose = best
            flag = 1
        else:
            flag = 0
        return flag, revisitPose
