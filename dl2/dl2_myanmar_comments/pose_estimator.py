#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import math

from pose2d import Pose2D
from scan2d import Scan2D
from pose_optimizer import PoseOptimizer
from data_associator import DataAssociator


class PoseEstimatorICP:
    def __init__(self, curScan=None, usedNum=0, pnrate=0., popt=None, dass=None):
        self.curScan = curScan if curScan else Scan2D()
        self.usedNum = usedNum
        self.pnrate = pnrate
        self.popt = popt if popt else PoseOptimizer()
        self.dass = dass if dass else DataAssociator()

    def getPnrate(self):
        return self.pnrate

    def getUsedNum(self):
        return self.usedNum

    def setScanPair_scan2d_GT(self, c, r):
        self.curScan = c
        self.dass.setRefBaseGT(r.lps)  # データ対応づけのために参照スキャン点を登録

    def setScanPair_l_point2d_GT(self, c, refLps):
        self.curScan = c
        self.dass.setRefBaseGT(refLps)  # データ対応づけのために参照スキャン点を登録    

    # 初期値initPoseを与えてICPによりロボット位置の推定値estPoseを求める
    def estimatePose(self, initPose, estPose):
        evmin = math.inf  # コスト最小値. 初期値は大きく
        evthre = 0.000001 #コスト変化閾値. 変化量がこれ以下なら繰り返し終了
        self.popt.setEvthre(evthre*0.1) #この値は試行錯誤的に設定
        self.popt.setEvlimit(0.2)  # evlimitは外れ値の閾値[m]
        ev = 0.  # コスト
        evold = evmin  # 1つ前の値. 収束判定のために使う
        pose = initPose
        poseMin = initPose
        for i in range(100):  # i<100は振動対策，ただしi=200など繰り返し数を多くすると質が上がる場合もある
            if i > 0:
                evold = ev
            mratio, pose = self.dass.findCorrespondenceGT(self.curScan, pose)  # データ対応づけ
            newPose = Pose2D()
            self.popt.setPoints(self.dass.curLps, self.dass.refLps)  # 対応結果を渡す
            ev, newPose = self.popt.optimizePoseSL(pose, newPose)  # その対応づけにおいてロボット位置の最適化
            pose = newPose
            if ev < evmin:  # コスト最小結果を保存
                poseMin = newPose
                evmin = ev
            if evold > ev and evold - ev < evthre:
                break
            elif ev > evold  and ev - evold < evthre:
                break
        self.pnrate = self.popt.getPnrate()
        self.usedNum = len(self.dass.curLps)
        estPose = poseMin
        return evmin, estPose
