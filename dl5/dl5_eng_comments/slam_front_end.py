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


# SLAMフロントエンド. ロボット位置推定, 地図生成, ループ閉じ込みを取り仕切る.
class SlamFrontEnd:
    def __init__(self, cnt=0, keyframeSkip=10, smat=None, lpd=None): # RasPiMouse
        self.cnt = cnt  # 論理時刻
        self.keyframeSkip = keyframeSkip  # キーフレーム間隔
        self.pcmap = PointCloudMap()  # 点群地図
        self.pg = PoseGraph()  # ポーズグラフ
        self.smat = smat if smat else ScanMatcher2D()  # スキャンマッチング
        self.lpd = lpd if lpd else LoopDetector()  # ループ検出器
        self.sback = SlamBackEnd()  # SLAMバックエンド
        self.sback.setPoseGraph(self.pg)

    def setPointCloudMap(self, p):
        self.pcmap = p

    def setRefScanMaker(self, r):
        self.smat.setRefScanMaker(r)

    def setDgCheck(self, p):
        self.smat.setDgCheck(p)

    def initialize(self):  # 初期化
        self.smat.reset()
        self.smat.setPointCloudMap(self.pcmap)
        self.sback.setPointCloudMap(self.pcmap)
        self.sback.setPoseGraph(self.pg)

    # 現在スキャンscanを処理する。
    def process(self, scan):
        if self.cnt == 0:
            self.initialize()  # 開始時に初期化
        self.smat.setDgCheck(True)  # 退化処理をする場合は活かす True
        #self.smat.setDgCheck(False) #退化処理をしない場合 False
        self.smat.matchScan(scan)  # スキャンマッチング
        curPose = self.pcmap.getLastPose()  # スキャンマッチングで推定した現在のロボット位置
        # ポーズグラフにオドメトリアークを追加
        if self.cnt == 0:  # 最初はノードを置くだけ
            self.pg.addNode(curPose)
        else:  # 次からはノードを追加して,オドメトリアークを張る
            cov = self.smat.getCovariance()
            self.makeOdometryArc(curPose, cov)
        if self.cnt % self.keyframeSkip == 0:  # キーフレームのときだけ行う
            self.pcmap.makeGlobalMap()  # 点群地図の全体地図を生成
        # キーフレームのときだけループ閉じ込み
        if self.cnt > self.keyframeSkip and (self.cnt % self.keyframeSkip) == 0:
            self.lpd.setPoseEstimator(self.smat.estim)
            self.lpd.setPoseFuser(self.smat.pfu)
            self.lpd.setDataAssociator(self.smat.pfu.dass)
            self.lpd.setPointCloudMap(self.pcmap)
            self.lpd.setPoseGraph(self.pg)

            flag = self.lpd.detectLoop(scan, curPose, self.cnt)  # ループ検出を起動
            if flag:
                self.sback.setPointCloudMap(self.pcmap)
                self.sback.setPoseGraph(self.pg)
                self.sback.adjustPoses()  # ループが見つかったらポーズ調整
                self.sback.remakeMaps()  # 地図やポーズグラフの修正
        self.cnt += 1

    # オドメトリアークの生成
    def makeOdometryArc(self, curPose, fusedCov):
        if len(self.pg.nodes) == 0:  # 念のためのチェック
            return False
        lastNode = self.pg.nodes[-1]  # 直前ノード
        curNode = self.pg.addNode(curPose)  # ポーズグラフに現在ノードを追加

        # 直前ノードと現在ノードの間にオドメトリアークを張る
        lastPose = lastNode.pose
        relPose = Pose2D()
        relPose = curPose.calRelativePose(lastPose, relPose)
        cov = np.eye(3)
        cov = CovarianceCalculator.rotateCovariance(lastPose, fusedCov, cov, True)  # 移動量の共分散に変換
        arc = self.pg.makeArc(lastNode.nid, curNode.nid, relPose, cov)  # アークの生成
        self.pg.addArc(arc)  # ポーズグラフにアークを追加
        return True
