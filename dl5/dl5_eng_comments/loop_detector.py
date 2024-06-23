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

from my_util import MyUtil
from pose2d import Pose2D
from scan2d import Scan2D
from pose_graph import PoseGraph
from point_cloud_map import PointCloudMap
from cost_function import CostFunction
from data_associator import DataAssociator
from pose_estimator import PoseEstimatorICP
from pose_fuser import PoseFuser
from covariance_calculator import CovarianceCalculator

# ループアーク設定情報
class LoopInfo:
    def __init__(self, arcked=False, curId=-1, refId=-1, pose=None, score=-1., cov=None):
        self.arcked = arcked  # すでにポーズアークを張ったか
        self.curId = curId  # 現在キーフレームid（スキャン）
        self.refId = refId  # 参照キーフレームid（スキャン，または，LocalGridMap2D）
        # 現在キーフレームが参照キーフレームにマッチするグローバル姿勢（Gridベースの場合は逆）
        self.pose = pose if pose else Pose2D()
        self.score = score  # ICPマッチングスコア
        self.cov = cov if cov else np.zeros((3, 3))  # 共分散

    def setArcked(self, t):
        self.arcked = t

class LoopDetector:
    def __init__(self, pg=None, radius=1., atdthre=5., scthre=0.2):
        self.pg = pg if pg else PoseGraph()  # ポーズグラフ
        self.radius = radius  # 探索半径[m]（現在位置と再訪点の距離閾値）
        self.atdthre = atdthre  # 累積走行距離の差の閾値[m]
        self.atdthre2 = 1.  # 直前のLoopDetec後からの走行距離の閾値[m]
        self.prevDetectionPose = Pose2D(math.inf, math.inf)  # 直前のLoopDetec時のpose
        self.scthre = scthre  # ICPスコアの閾値
        self.pcmap = PointCloudMap()  # 点群地図
        self.cfunc = CostFunction()  # コスト関数(ICPとは別に使う)
        self.estim = PoseEstimatorICP()  # ロボット位置推定器(ICP)
        self.dass = DataAssociator()  # データ対応づけ器
        self.pfu = PoseFuser()  # センサ融合器

    def setPoseGraph(self, p):
        self.pg = p

    def setPoseEstimator(self, p):
        self.estim = p

    def setPoseFuser(self, p):
        self.pfu = p

    def setDataAssociator(self, d):
        self.dass = d

    def setPointCloudMap(self, p):
        self.pcmap = p

    # ループ検出
    # 現在位置curPoseに近く, 現在スキャンcurScanに形が一致する場所をロボット軌跡から見つけてポーズアークを張る
    def detectLoop(self, curScan, curPose, cnt):
        print("-- detectLoop -- ")
        # 最も近い部分地図を探す
        atd = self.pcmap.atd  # 現在の実際の累積走行距離
        atdR = 0  # 下記の処理で軌跡をなぞる時の累積走行距離
        poses = self.pcmap.poses  # ロボット軌跡
        dmin = math.inf  # 前回訪問点までの距離の最小値
        imin = 0
        jmin = 0  # 距離最小の前回訪問点のインデックス
        prevP = Pose2D()  # 直前のロボット位置
        len_self_pcmap_submaps_1 = len(self.pcmap.submaps) - 1
        math_sqrt = math.sqrt
        atdFromPrev = (curPose.tx - self.prevDetectionPose.tx) * (curPose.tx - self.prevDetectionPose.tx) + (curPose.ty - self.prevDetectionPose.ty) * (curPose.ty - self.prevDetectionPose.ty)
        if atdFromPrev < self.atdthre2:  # 直前のループ後の走行距離が短いときはループ検出しない
            print("Already Loop Detected: dis=%f, (x,y)=%f %f"%(atdFromPrev,self.prevDetectionPose.tx,self.prevDetectionPose.ty))
            return False
        for i in range(0, len_self_pcmap_submaps_1, 1):  # 現在の部分地図以外を探す
            submap = self.pcmap.submaps[i]  # i番目の部分地図
            for j in range(submap.cntS, submap.cntE, 1):  # 部分地図の各ロボット位置について
                p = poses[j]  # ロボット位置
                atdR += math_sqrt((p.tx - prevP.tx) * (p.tx - prevP.tx) + (p.ty - prevP.ty) * (p.ty - prevP.ty))
                if atd - atdR < self.atdthre:  # 現在位置までの走行距離が短いとループとみなさず, もうやめる
                    i = len(self.pcmap.submaps)  # これで外側のループからも抜ける
                    break
                prevP = p
                d = (curPose.tx - p.tx) * (curPose.tx - p.tx) + (curPose.ty - p.ty) * (curPose.ty - p.ty)
                if d < dmin:  # 現在位置とpとの距離がこれまでの最小か
                    dmin = d
                    imin = i  # 候補となる部分地図のインデックス
                    jmin = j  # 前回訪問点のインデックス
        print("dmin=%f, radius=%f, imin=%d, jmin=%d atd=%d atdR=%d" % (math.sqrt(dmin), self.radius, imin, jmin, self.pcmap.atd, atdR))  # 確認用
        if dmin > self.radius * self.radius:  # 前回訪問点までの距離が遠いとループ検出しない
            return False
        refSubmap = self.pcmap.submaps[imin]  # 最も近い部分地図を参照スキャンにする
        initPose = poses[jmin]

        # 再訪点の位置を求める
        revisitPose = Pose2D()
        flag, revisitPose = self.estimateRevisitPose(curScan, refSubmap.mps, curPose, revisitPose)

        if flag:  # ループを検出した
            icpCov = np.empty([3, 3])  # ICPの共分散
            icpCov = self.pfu.calIcpCovariance(revisitPose, curScan, icpCov)  # ICPの共分散を計算
            info = LoopInfo()  # ループ検出結果
            info.pose = revisitPose  # ループアーク情報に再訪点位置を設定
            info.cov = icpCov  # ループアーク情報に共分散を設定。
            info.curId = cnt  # 現在位置のノードid
            info.refId = int(jmin)  # 前回訪問点のノードid
            self.makeLoopArc(info)  # ループアーク生成
            self.prevDetectionPose = revisitPose #一度検出したらatdthre2の間，検出しないようにするため

        return flag

    # 前回訪問点(refId)を始点ノード、現在位置(curId)を終点ノードにして、ループアークを生成する。
    def makeLoopArc(self, info):
        if info.arcked:  # infoのアークはすでに張ってある
            return
        info.setArcked(True)
        srcPose = self.pcmap.poses[info.refId]  # 前回訪問点の位置
        dstPose = Pose2D(info.pose.tx, info.pose.ty, info.pose.th)  # 再訪点の位置
        relPose = Pose2D()
        relPose = dstPose.calRelativePose(srcPose, relPose)  # ループアークの拘束
        # アークの拘束は始点ノードからの相対位置なので, 共分散をループアークの始点ノード座標系に変換
        cov = np.empty([3, 3])
        cov = CovarianceCalculator.rotateCovariance(srcPose, info.cov, cov, True)  # 共分散の逆回転
        arc = self.pg.makeArc(info.refId, info.curId, relPose, cov)  # ループアーク生成
        self.pg.addArc(arc)  # ループアーク登録

    # 現在スキャンcurScanと部分地図の点群refLpsでICPを行い, 再訪点の位置を求める。
    def estimateRevisitPose(self, curScan, refLps, initPose, revisitPose):
        self.dass.setRefBaseGT(refLps)  # データ対応づけ器に参照点群を設定
        self.cfunc.setEvlimit(0.2)  # コスト関数の誤差閾値
        print("initPose: tx=%f, ty=%f, th=%f" % (initPose.tx, initPose.ty, initPose.th))  # 確認用
        usedNumMin = 50  # 100
        # 初期位置initPoseの周囲をしらみつぶしに調べる
        # 効率化のためICPは行わず, 各位置で単純にマッチングスコアを調べる
        rangeT = 0.5 #org 1. # 並進の探索範囲[m]
        rangeA = 25. #org 45.  # 回転の探索範囲[度]
        dd = 0.2  # 並進の探索間隔[m]
        da = 2.  # 回転の探索間隔[度]
        scoreMin = 1000.
        scores = np.empty(0)
        candidates = np.empty(0)  # スコアのよい候補位置
        for dy in np.arange(-rangeT, rangeT + dd, dd):  # 並進yの探索繰り返し
            y = initPose.ty + dy  # 初期位置に変位分dyを加える
            for dx in np.arange(-rangeT, rangeT + dd, dd):  # 並進xの探索繰り返し
                x = initPose.tx + dx  # 初期位置に変位分dxを加える
                for dth in np.arange(-rangeA, rangeA + da, da):  # 回転の探索繰り返し
                    th = MyUtil.add(initPose.th, dth)  # 初期位置に変位分dthを加える
                    pose = Pose2D(x, y, th)
                    mratio, pose = self.dass.findCorrespondenceGT(curScan, pose)  # 位置poseでデータ対応づけ
                    usedNum = len(self.dass.curLps)
                    if usedNum < usedNumMin or mratio < 0.9:  # 対応率が悪いと飛ばす
                        continue
                    self.cfunc.setPoints(self.dass.curLps, self.dass.refLps)  # コスト関数に点群を設定
                    score = self.cfunc.calValuePD(x, y, th)  # コスト値（マッチングスコア）
                    pnrate = self.cfunc.getPnrate()  # 詳細な点の対応率
                    if pnrate > 0.8:
                        candidates = np.append(candidates, pose)
                        if score < scoreMin:
                            scoreMin = score
                        scores = np.append(scores, score)
        len_candidates = len(candidates)
        print("candidates.size=%d" % len_candidates)  # 確認用
        if len_candidates == 0:
            flag = 0
            return flag, revisitPose

        # 候補位置candidatesの中から最もよいものをICPで選ぶ
        best = Pose2D()  # 最良候補
        smin = 1000000.  # ICPスコア最小値
        self.estim.setScanPair_l_point2d_GT(curScan, refLps)  # ICPにスキャン設定
        for i in range(len_candidates):
            p = candidates[i]  # 候補位置
            print("candidates %d (%d)" % (i, len_candidates))  # 確認用
            estP = Pose2D()
            score, estP = self.estim.estimatePose(p, estP)  # ICPでマッチング位置を求める
            pnrate = self.estim.getPnrate()  # ICPでの点の対応率
            usedNum = self.estim.getUsedNum()  # ICPで使用した点数
            print("score=%f, pnrate=%f, usedNum=%d" % (score, pnrate, usedNum))  # 確認用
            if score < smin and pnrate >= 0.9 and usedNum >= usedNumMin:  # ループ検出は条件厳しく
                smin = score
                best = estP

        # 最小スコアが閾値より小さければ見つけた
        if smin <= self.scthre:
            revisitPose = best
            flag = 1
        else:
            flag = 0
        return flag, revisitPose
