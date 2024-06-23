#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import numpy as np

from point_cloud_map import PointCloudMap
from pose_graph import PoseGraph
from slam_back_end import SlamBackEnd
from covariance_calculator import CovarianceCalculator
from loop_detector import LoopDetector
from scan_matcher2d import ScanMatcher2D
from pose2d import Pose2D


# SLAM Front End. Robot position estimate, Map generation, Loop closing the loop.
class SlamFrontEnd:
    def __init__(self, cnt=0, keyframeSkip=10, smat=None, lpd=None): # RasPiMouse
        self.cnt = cnt  # Logical time
        self.keyframeSkip = keyframeSkip  # Key frame interval
        self.pcmap = PointCloudMap()  # Point group map
        self.pg = PoseGraph()  # Pose graph
        self.smat = smat if smat else ScanMatcher2D()  # Scan matching
        self.lpd = lpd if lpd else LoopDetector()  # Loop detector
        self.sback = SlamBackEnd()  # SLAM backend
        self.sback.setPoseGraph(self.pg)

    def setPointCloudMap(self, p):
        self.pcmap = p

    def setRefScanMaker(self, r):
        self.smat.setRefScanMaker(r)

    def setDgCheck(self, p):
        self.smat.setDgCheck(p)

    def initialize(self):  # initialization
        self.smat.reset()
        self.smat.setPointCloudMap(self.pcmap)
        self.sback.setPointCloudMap(self.pcmap)
        self.sback.setPoseGraph(self.pg)

    # currentlyProcessingScanScanScan
    def process(self, scan):
        if self.cnt == 0:
            self.initialize()  # initializeAtTheStart
        self.smat.setDgCheck(True)  # useItForDegenerationProcessing True
        #self.smat.setDgCheck(False) #ifYouDoNotPerformDegeneration False
        self.smat.matchScan(scan)  # scanMatching
        curPose = self.pcmap.getLastPose()  # currentRobotPositionEstimatedByScanMatching
        # addedOdometrikToThePoseGraph
        if self.cnt == 0:  # atFirst,JustPutTheNode
            self.pg.addNode(curPose)
        else:  # addANodeFromNextTime,odometrik
            cov = self.smat.getCovariance()
            self.makeOdometryArc(curPose, cov)
        if self.cnt % self.keyframeSkip == 0:  # doItOnlyForKeyFrames
            self.pcmap.makeGlobalMap()  # generateTheWholeMapOfThePointGroupMap
        # Loop is closed only for key frames
        if self.cnt > self.keyframeSkip and (self.cnt % self.keyframeSkip) == 0:
            self.lpd.setPoseEstimator(self.smat.estim)
            self.lpd.setPoseFuser(self.smat.pfu)
            self.lpd.setDataAssociator(self.smat.pfu.dass)
            self.lpd.setPointCloudMap(self.pcmap)
            self.lpd.setPoseGraph(self.pg)

            flag = self.lpd.detectLoop(scan, curPose, self.cnt)  # Start the loop detection
            if flag:
                self.sback.setPointCloudMap(self.pcmap)
                self.sback.setPoseGraph(self.pg)
                self.sback.adjustPoses()  # If you find a loop, adjust the pose
                self.sback.remakeMaps()  # Map and pose graph correction
        self.cnt += 1

    # Production of Odometrik
    def makeOdometryArc(self, curPose, fusedCov):
        if len(self.pg.nodes) == 0:  # Check just in case
            return False
        lastNode = self.pg.nodes[-1]  # Just before
        curNode = self.pg.addNode(curPose)  # Currently add nodes to pose graphs

        # Put an Odometrik between the node and the present node just before
        lastPose = lastNode.pose
        relPose = Pose2D()
        relPose = curPose.calRelativePose(lastPose, relPose)
        cov = np.eye(3)
        cov = CovarianceCalculator.rotateCovariance(lastPose, fusedCov, cov, True)  # Convert to coordinates of moving amount
        arc = self.pg.makeArc(lastNode.nid, curNode.nid, relPose, cov)  # Arc generation
        self.pg.addArc(arc)  # Add an arc to the pose graph
        return True
