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

from l_point2d import LPoint2D, ptype
from pose2d import Pose2D
from scan2d import Scan2D
from point_cloud_map import PointCloudMap
from covariance_calculator import CovarianceCalculator
from ref_scan_maker import RefScanMaker
from scan_point_resampler import ScanPointResampler
from scan_point_analyser import ScanPointAnalyser
from pose_estimator import PoseEstimatorICP
from pose_fuser import PoseFuser


# ICPを用いてスキャンマッチングを行う
class ScanMatcher2D:
    def __init__(
        self,
        cnt=-1,
        prevScan=None,
        initPose=None,
        scthre=1.0,
        nthre=50,
        dgcheck=False,
        pcmap=None,
        spres=None,
        spana=None,
        estim=None,
        rsm=None,
        pfu=None,
        cov=None,
    ):
        self.cnt = cnt  # 論理時刻. スキャン番号に対応
        self.prevScan = prevScan if prevScan else Scan2D()  # 1つ前のスキャン
        self.initPose = initPose if initPose else Pose2D()  # 地図の原点の位置. 通常(0,0,0)
        self.scthre = scthre  # スコア閾値. これより大きいとICP失敗とみなす
        self.nthre = nthre  # 使用点数閾値. これより小さいとICP失敗とみなす
        self.dgcheck = dgcheck  # 退化処理をするか
        self.pcmap = pcmap if pcmap else PointCloudMap()  # 点群地図
        self.spres = spres if spres else ScanPointResampler()  # スキャン点間隔均一化
        self.spana = spana if spana else ScanPointAnalyser()  # スキャン点法線計算
        self.estim = estim if estim else PoseEstimatorICP()  # ロボット位置推定器
        self.rsm = rsm if rsm else RefScanMaker()  # 参照スキャン生成
        self.pfu = pfu if pfu else PoseFuser()  # センサ融合器
        self.cov = cov if cov else np.eye(3)  # ロボット移動量の共分散行列

    def setPoseEstimator(self, p):
        self.estim = p

    def setPoseFuser(self, p):
        self.pfu = p

    def setRefScanMaker(self, r):
        self.rsm = r
        if len(self.pcmap) != 0:
            self.rsm.setPointCloudMap(self.pcmap)

    def setPointCloudMap(self, m):
        self.pcmap = m
        self.rsm.setPointCloudMap(self.pcmap)

    def reset(self):
        self.cnt = -1

    def setDgCheck(self, t):
        self.dgcheck = t

    def getCovariance(self):
        return self.cov

    # スキャンマッチングの実行
    def matchScan(self, curScan):
        self.cnt = self.cnt + 1
        self.spres.resamplePoints(curScan)  # spresが設定されていればスキャン点間隔を均一化する
        self.spana.analysePoints(curScan.lps)  # spanaが設定されていればスキャン点の法線を計算する
        # 最初のスキャンは単に地図に入れるだけ
        if self.cnt == 0:
            self.growMap(curScan, self.initPose)
            self.prevScan = curScan  # 直前スキャンの設定
            return True
        # Scanに入っているオドメトリ値を用いて移動量を計算する
        odoMotion = Pose2D()  # オドメトリに基づく移動量
        odoMotion = curScan.pose.calRelativePose(self.prevScan.pose, odoMotion)  # 前スキャンとの相対位置が移動量
        lastPose = self.pcmap.getLastPose()  # 直前位置
        predPose = Pose2D()  # オドメトリによる予測位置
        predPose = Pose2D.calGlobalPose(odoMotion, lastPose, predPose)  # 直前位置に移動量を加えて予測位置を得る
        refScan = self.rsm.makeRefScanLM()  # 参照スキャンの生成 地図の点群を用いる
        
        self.estim.setScanPair_scan2d_GT(curScan, refScan)  # ICPにスキャンを設定

        estPose = Pose2D()  # ICPによる推定位置
        score, estPose = self.estim.estimatePose(predPose, estPose)  # 予測位置を初期値にしてICPを実行
        usedNum = self.estim.getUsedNum()

        # スキャンマッチングに成功したかどうか
        if score <= self.scthre and usedNum >= self.nthre:  # スコアが閾値より小さければ成功とする
            successful = True
        else:
            successful = False

        if self.dgcheck:  # 退化の対処をする場合
            if successful:
                fusedPose = Pose2D()  # 融合結果
                fusedCov = np.eye(3)  # センサ融合後の共分散
                self.pfu.setRefScan(refScan)
                # センサ融合器pfuで、ICP結果とオドメトリ値を融合する
                estPose, self.cov = self.pfu.fusePose(curScan, estPose, odoMotion, lastPose, fusedPose, fusedCov)
            else:  # ICP成功でなければオドメトリによる予測位置を使う
                estPose = predPose
                self.cov = self.pfu.calOdometryCovariance(odoMotion, lastPose, self.cov)  # covはオドメトリ共分散だけ
        else:
            if not successful:
                estPose = predPose
        self.growMap(curScan, estPose)  # 地図にスキャン点群を追加
        self.prevScan = copy.deepcopy(curScan)  # 直前スキャンの設定

        return successful

    # 現在スキャンを追加して、地図を成長させる
    def growMap(self, scan, pose):
        lps = scan.lps  # スキャン点群(ロボット座標系)
        R = pose.Rmat  # 推定したロボット位置
        tx = pose.tx
        ty = pose.ty

        scanG_list = list()
        for i in range(len(lps)):
            lp = lps[i]
            if lp.type == ptype.ISOLATE:  # 孤立点（法線なし）は除外
                continue
            x = R[0, 0] * lp.x + R[0, 1] * lp.y + tx  # 地図座標系に変換
            y = R[1, 0] * lp.x + R[1, 1] * lp.y + ty
            nx = R[0, 0] * lp.nx + R[0, 1] * lp.ny  # 法線ベクトルも変換
            ny = R[1, 0] * lp.nx + R[1, 1] * lp.ny

            mlp = LPoint2D(self.cnt, x, y)  # 新規に点を生成
            mlp.setNormal(nx, ny)
            mlp.setType(lp.type)
            scanG_list.append(mlp)
        scanG = np.asarray(scanG_list)

        # 点群地図pcmapに登録
        self.pcmap.addPose(pose)
        self.pcmap.addPoints(scanG)
        self.pcmap.setLastScan(scan)  # 参照スキャン用に保存
        self.pcmap.makeLocalMap()  # 局所地図を生成
        self.pcmap.setLastPose(pose)
