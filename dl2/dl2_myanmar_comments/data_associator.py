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
import copy

from l_point2d import LPoint2D
from nn_grid_table import NNGridTable


class DataAssociator:
    def __init__(self, curLps=None, refLps=None, baseLps=None, nntab=None):
        self.curLps = curLps if curLps else np.array([LPoint2D()])  # 対応がとれた現在スキャンの点群
        self.refLps = refLps if refLps else np.array([LPoint2D()])  # 対応がとれた参照スキャンの点群
        self.baseLps = baseLps if baseLps else np.empty(0)  # 参照スキャンの点を格納しておく. LS作業用
        self.nntab = nntab if nntab else NNGridTable()

    # 参照スキャンの点rlpsをポインタにしてnntabに入れる
    def setRefBaseGT(self, rlps):
        self.nntab.clear()
        for i in range(len(rlps)):
            self.nntab.addPoint(rlps[i])  # ポインタにして格納

    # 現在スキャンcurScanの各スキャン点をpredPoseで座標変換した位置に最も近い点を見つける
    def findCorrespondenceGT(self, curScan, predPose):
        curLps_list = list()
        refLps_list = list()

        for i in range(len(curScan.lps)):
            clp = curScan.lps[i]  # 現在スキャンの点
            # 格子テーブルにより最近傍点を求める. 格子テーブル内に距離閾値dthreがあることに注意
            rlp = self.nntab.findClosestPoint(clp, predPose)
            if rlp:
                curLps_list.append(clp)
                refLps_list.append(rlp)
        self.curLps = np.asarray(curLps_list)
        self.refLps = np.asarray(refLps_list)
        ratio = (1.0 * len(self.curLps) / len(curScan.lps))  # 対応がとれた点の比率
        return ratio, predPose
