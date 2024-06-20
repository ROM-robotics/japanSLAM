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


# SLAMフロントエンド. ロボット位置推定, 地図生成, ループ閉じ込みを取り仕切る.
class SlamFrontEnd:
    def __init__(self, cnt=0, keyframeSkip=10, smat=None): # RasPiMouse        
        self.cnt = cnt  # 論理時刻
        self.keyframeSkip = keyframeSkip  # キーフレーム間隔
        self.pcmap = PointCloudMap()  # 点群地図
        self.smat = smat if smat else ScanMatcher2D()  # スキャンマッチング

    def setPointCloudMap(self, p):
        self.pcmap = p

    def setRefScanMaker(self, r):
        self.smat.setRefScanMaker(r)

    def setDgCheck(self, p):
        self.smat.setDgCheck(p)

    def initialize(self):  # 初期化
        self.smat.reset()
        self.smat.setPointCloudMap(self.pcmap)

    # 現在のスキャンデータscanを処理する
    def process(self, scan):
        if self.cnt == 0:
            self.initialize()  # 開始時に初期化
        self.smat.setDgCheck(True)  # 退化処理をする場合 True
        #self.smat.setDgCheck(False) # 退化処理をしない場合 False
        self.smat.matchScan(scan)  # スキャンマッチング

        if self.cnt % self.keyframeSkip == 0:  # キーフレームのときだけ行う
            self.pcmap.makeGlobalMap()  # 点群地図の全体地図を生成
        
        self.cnt += 1
