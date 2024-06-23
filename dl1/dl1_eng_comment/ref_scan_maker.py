#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import numpy as np

from l_point2d import LPoint2D
from scan2d import Scan2D
from point_cloud_map import PointCloudMap


class RefScanMaker:
    def __init__(self, pcmap=None, refScan=None):
        self.pcmap = pcmap if pcmap else PointCloudMap()  # Point group map
        self.refScan = refScan if refScan else Scan2D()  # Reference scan body. Providing this outside

    def setPointCloudMap(self, p):
        self.pcmap = p

    def makeRefScanLM(self):
        localMap = self.pcmap.localMap  # Local map of the point group map
        refLps_list = list()
        for i in range(len(localMap)):
            rp = localMap[i]
            refLps_list.append(rp)
        self.refScan.lps = np.asarray(refLps_list)
        return self.refScan
