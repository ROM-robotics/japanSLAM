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
from p2o_driver2d import P2oDriver2D


class SlamBackEnd:
    def __init__(self, newPoses=None, pcmap=None, pg=None):
        self.newPoses = newPoses if newPoses else np.empty(0)  # ポーズ調整後の姿勢
        self.pcmap = pcmap if pcmap else PointCloudMap()  # 点群地図
        self.pg = pg if pg else PoseGraph()  # ポーズグラフ

    def setPointCloudMap(self, pcmap):
        self.pcmap = pcmap

    def setPoseGraph(self, pg):
        self.pg = pg

    def adjustPoses(self):
        self.newPoses = np.empty(0)  # 初期化
        p2o = P2oDriver2D()
        self.newPoses = p2o.doP2o(self.pg, self.newPoses, 5)  # 5回くり返す
        return self.newPoses[-1]

    def remakeMaps(self):  # PoseGraphの修正
        pnodes = self.pg.nodes
        num = len(self.newPoses)
        for i in range(num):
            npose = self.newPoses[i]
            pnode = pnodes[i]  # ノードはロボット位置と1:1対応
            pnode.setPose(npose)  # 各ノードの位置を更新
        self.pcmap.remakeMaps(self.newPoses)  # PointCloudMapの修正
