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

from pose2d import Pose2D
from scan2d import Scan2D
from nn_grid_table import NNGridTable


# 部分地図
class Submap:
    def __init__(self, atdS=0.0, cntS=0, cntE=-1, mps=None):
        self.atdS = atdS  # 部分地図の始点での累積走行距離
        self.cntS = cntS  # 部分地図の最初のスキャン番号
        self.cntE = cntE  # 部分地図の最後のスキャン番号
        self.mps = mps if mps else np.empty(0)          # 部分地図内のスキャン点群 vector<LPoint2D>

    def addPoints(self, lps):
        mps_list = self.mps.tolist()
        for i in range(len(lps)):
            mps_list.append(lps[i])
        self.mps = np.asarray(mps_list)

    # 格子テーブルを用いて、部分地図の代表点を得る
    def subsamplePoints(self, nthre):
        nntab = NNGridTable()  # 格子テーブル
        for i in range(len(self.mps)):
            lp = self.mps[i]
            nntab.addPoint(lp)  # 全点を登録
        sps = np.empty(0)
        sps = nntab.makeCellPoints(nthre, sps)  # nthre個以上のセルの代表点をspsに入れる
        return sps

# 点群地図の基底クラス
class PointCloudMap:
    MAX_POINT_NUM = 1000000  # globalMapの最大点数

    def __init__(
        self,
        nthre=5, # nthre=0,
        poses=None,
        lastPose=None,
        lastScan=None,
        globalMap=None,
        localMap=None,
        nntab=None,
        atdThre=5.,
        atd=0.,
        submaps=None
    ):
        self.nthre = nthre  # 格子テーブルセル点数閾値(GTとLPのみ）
        # ロボット軌跡
        self.poses = poses if poses else np.empty([0, 0])
        # 最後に推定したロボット位置
        self.lastPose = lastPose if lastPose else Pose2D()
        # 最後に処理したスキャン
        self.lastScan = lastScan if lastScan else Scan2D()
        # 全体地図. 間引き後の点
        self.globalMap = globalMap if globalMap else np.empty(self.MAX_POINT_NUM)
        # 現在位置近傍の局所地図. スキャンマッチングに使う
        self.localMap = localMap if localMap else np.empty(0)
        # for 格子テーブルを用いた点群地図
        self.nntab = nntab if nntab else NNGridTable()
        # 部分地図の区切りとなる累積走行距離(atd)[m]        
        self.atdThre = atdThre
        # 現在の累積走行距離(accumulated travel distance)        
        self.atd = atd
        # 部分地図
        self.submaps = submaps if submaps else np.array([Submap()])

    def setLastPose(self, pose2d):
        self.lastPose = copy.deepcopy(pose2d)

    def getLastPose(self):
        return self.lastPose

    def setLastScan(self, scan2d):
        self.lastScan = copy.deepcopy(scan2d)

    # ロボット位置の追加
    def addPose(self, pose2d):
        # 累積走行距離(atd)の計算
        if len(self.poses) > 0:
            pp = self.poses[-1]
            self.atd = self.atd + math.sqrt((pose2d.tx - pp.tx) * (pose2d.tx - pp.tx) + (pose2d.ty - pp.ty) * (pose2d.ty - pp.ty))
        else:
            self.atd = self.atd + math.sqrt(pose2d.tx * pose2d.tx + pose2d.ty * pose2d.ty)
        self.poses = np.append(self.poses, copy.deepcopy(pose2d))

    # スキャン点群の追加 LP    
    def addPoints(self, vector_l_point2d):
        curSubmap = self.submaps[-1]  # 現在の部分地図
        if self.atd - curSubmap.atdS >= self.atdThre:  # 累積走行距離が閾値を超えたら新しい部分地図に変える
            size = len(self.poses)
            curSubmap.cntE = size - 1  # 部分地図の最後のスキャン番号
            curSubmap.mps = curSubmap.subsamplePoints(self.nthre)  # 終了した部分地図は代表点のみにする（軽量化）
            submap = Submap(self.atd, size)  # 新しい部分地図
            submap.addPoints(vector_l_point2d)  # スキャン点群の登録
            self.submaps = np.append(self.submaps, submap)  # 部分地図を追加
        else:
            curSubmap.addPoints(vector_l_point2d)  # 現在の部分地図に点群を追加

    # 全体地図の生成. 局所地図もここでいっしょに作った方が速い LP
    def makeGlobalMap(self):
        self.globalMap = np.empty(self.MAX_POINT_NUM)  # 初期化
        self.localMap = np.empty(0)  # 初期化
        globalMap_list = list()
        localMap_list = list()

        # 現在以外のすでに確定した部分地図から点を集める
        num = len(self.submaps) - 1
        for i in range(num):
            submap = self.submaps[i]  # 部分地図
            mps = submap.mps  # 部分地図の点群. 代表点だけになっている
            num2 = len(mps)
            for j in range(num2):
                globalMap_list.append(mps[j])  # 全体地図には全点入れる
            if i == len(self.submaps) - 2:  # 局所地図には最後の部分地図だけ入れる
                for j in range(num2):
                    localMap_list.append(mps[j])

        # 現在の部分地図の代表点を全体地図と局所地図に入れる
        curSubmap = self.submaps[-1]  # 現在の部分地図
        sps = curSubmap.subsamplePoints(self.nthre)  # 代表点を得る
        for i in range(len(sps)):
            globalMap_list.append(sps[i])  # 全体地図には全点入れる
            localMap_list.append(sps[i])
        self.globalMap = np.asarray(globalMap_list)
        self.localMap = np.asarray(localMap_list)

    # 局所地図生成 LP
    def makeLocalMap(self):
        self.localMap = np.empty(0)  # 初期化
        localMap_list = list()
        if len(self.submaps) >= 2:
            submap = self.submaps[len(self.submaps) - 2]  # 直前の部分地図だけ使う
            mps = submap.mps  # 部分地図の点群. 代表点だけになっている
            num = len(mps)
            localMap_list = [mps[i] for i in range(num)]
        # 現在の部分地図の代表点を局所地図に入れる
        curSubmap = self.submaps[-1]  # 現在の部分地図
        sps = curSubmap.subsamplePoints(self.nthre)  # 代表点を得る
        for i in range(len(sps)):
            localMap_list.append(sps[i])
        self.localMap = np.asarray(localMap_list)

    # ポーズ調整後のロボット軌跡newPoseを用いて, 地図を再構築する
    def remakeMaps(self, newPoses):
        # 各部分地図内の点の位置を修正する
        for i in range(len(self.submaps)):
            submap = self.submaps[i]
            mps = submap.mps  # 部分地図の点群. 現在地図以外は代表点になっている
            for j in range(len(mps)):
                mp = mps[j]
                idx = mp.sid  # 点のスキャン番号
                if idx >= len(self.poses):  # 不正なスキャン番号（あったらバグ）
                    continue
                oldPose = self.poses[idx]  # mpに対応する古いロボット位置
                newPose = newPoses[idx]  # mpに対応する新しいロボット位置
                R1 = oldPose.Rmat
                R2 = newPose.Rmat
                lp1 = oldPose.relativePoint(mp)  # oldPoseでmpをセンサ座標系に変換
                lp2 = newPose.globalPoint(lp1)  # newPoseでポーズ調整後の地図座標系に変換
                mp.x = lp2.x
                mp.y = lp2.y
                nx = R1[0, 0] * mp.nx + R1[1, 0] * mp.ny  # 法線ベクトルもoldPoseでセンサ座標系に変換
                ny = R1[0, 1] * mp.nx + R1[1, 1] * mp.ny
                nx2 = R2[0, 0] * nx + R2[0, 1] * ny  # 法線ベクトルもnewPoseでポーズ調整後の地図座標系に変換
                ny2 = R2[1, 0] * nx + R2[1, 1] * ny
                mp.setNormal(nx2, ny2)
                self.submaps[i].mps[j] = mp
        self.makeGlobalMap()  # 部分地図から全体地図と局所地図を生成
        num = len(self.poses)
        self.poses = [newPoses[i] for i in range(num)]
        self.lastPose = copy.deepcopy(newPoses[-1])
