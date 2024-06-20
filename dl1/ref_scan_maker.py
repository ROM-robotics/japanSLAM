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
        self.pcmap = pcmap if pcmap else PointCloudMap()  # 点群地図
        self.refScan = refScan if refScan else Scan2D()  # 参照スキャン本体. これを外に提供

    def setPointCloudMap(self, p):
        self.pcmap = p

    def makeRefScanLM(self):
        localMap = self.pcmap.localMap  # 点群地図の局所地図
        refLps_list = list()
        for i in range(len(localMap)):
            rp = localMap[i]
            refLps_list.append(rp)
        self.refScan.lps = np.asarray(refLps_list)
        return self.refScan
