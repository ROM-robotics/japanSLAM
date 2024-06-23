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
        self.cnt = cnt  # Logical time
        self.keyframeSkip = keyframeSkip  # Key frame interval
        self.pcmap = PointCloudMap()  # Point group map
        self.smat = smat if smat else ScanMatcher2D()  # Scan matching

    def setPointCloudMap(self, p):
        self.pcmap = p

    def setRefScanMaker(self, r):
        self.smat.setRefScanMaker(r)

    def setDgCheck(self, p):
        self.smat.setDgCheck(p)

    def initialize(self):  # Initialization
        self.smat.reset()
        self.smat.setPointCloudMap(self.pcmap)

    # Process the current scandata data SCAN
    def process(self, scan):
        if self.cnt == 0:
            self.initialize()  # Initialize at the start
        self.smat.setDgCheck(True)  # When performing degeneration, True
        #self.smat.setDgCheck(False) # If you do not perform degeneration, False
        self.smat.matchScan(scan)  # Scan matching

        if self.cnt % self.keyframeSkip == 0:  # Do it only for key frames
            self.pcmap.makeGlobalMap()  # Generate the whole map of the point group map
        
        self.cnt += 1
