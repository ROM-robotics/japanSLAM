#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import numpy as np
import copy

from l_point2d import LPoint2D
from pose2d import Pose2D


# スキャンデータの構造（レーザレンジセンサデータとその時のオドメトリデータ）
class Scan2D:
    MAX_SCAN_RANGE = 3.5 #4.0(Raspberry Pi Mouse) 3.5(TurtleBot3)
    MIN_SCAN_RANGE = 0.12 #0.06(Raspberry Pi Mouse) 0.12(TurtleBot3)

    def __init__(self, sid=0, pose=None):
        self.sid = int(sid)  # スキャンid
        self.pose = pose if pose else Pose2D()  # スキャン取得時のオドメトリ値
        self.lps = np.array([LPoint2D()])  # スキャン点群 LPoint2D()

    def setSid(self, sid):
        self.sid = int(sid)

    def setLps(self, ps):
        self.lps = copy.deepcopy(ps)

    def setPose(self, p):
        self.pose = copy.deepcopy(p)
