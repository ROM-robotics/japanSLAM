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
from covariance_calculator import CovarianceCalculator
from scan_matcher2d import ScanMatcher2D
from pose2d import Pose2D


# SLAM Front -ended robot position estimation, map generation, loop closed.
class SlamFrontEnd:
    def __init__(self, cnt=0, keyframeSkip=10, smat=None): # RasPiMouse        
        self.cnt = cnt  # logicalTime
        self.keyframeSkip = keyframeSkip  # keyFrameInterval
        self.pcmap = PointCloudMap()  # pointGroupMap
        self.smat = smat if smat else ScanMatcher2D()  # scanMatching

    def setPointCloudMap(self, p):
        self.pcmap = p

    def setRefScanMaker(self, r):
        self.smat.setRefScanMaker(r)

    def setDgCheck(self, p):
        self.smat.setDgCheck(p)

    def initialize(self):  # initialization
        self.smat.reset()
        self.smat.setPointCloudMap(self.pcmap)

    # Process the current scandata SCAN
    def process(self, scan):    # ( လိုင်းနံပါတ် 40 )
        if self.cnt == 0:
            self.initialize()  # Initialize at the start
        self.smat.setDgCheck(True)  # When performing degeneration processing True
        #self.smat.setDgCheck(False) # If you do not perform degeneration False
        self.smat.matchScan(scan)  # Scan matching ( လိုင်းနံပါတ် 45 )

        if self.cnt % self.keyframeSkip == 0:  # Do it only for key frames
            self.pcmap.makeGlobalMap()  # Generate the whole map of the point group map ( လိုင်းနံပါတ် 47 )
        
        self.cnt += 1
