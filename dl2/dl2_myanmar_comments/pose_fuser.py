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

from my_util import RAD2DEG, DEG2RAD
from pose2d import Pose2D
from data_associator import DataAssociator
from covariance_calculator import CovarianceCalculator


# センサ融合器. ICPとオドメトリの推定値を融合する
class PoseFuser:
    def __init__(self, ecov=None, mcov=None, totalCov=None, dass=None, cvc=None):
        self.ecov = ecov if ecov else np.zeros((3, 3))  # ICPの共分散行列
        self.mcov = mcov if mcov else np.zeros((3, 3))  # オドメトリの共分散行列
        self.totalCov = totalCov if totalCov else np.zeros((3, 3))
        self.dass = dass if dass else DataAssociator()  # データ対応づけ器
        self.cvc = cvc if cvc else CovarianceCalculator()  # 共分散計算器

    def setDataAssociator(self, d):
        self.dass = d

    def setRefScan(self, refScan):
        self.dass.setRefBaseGT(refScan.lps)

    # ICPの共分散行列の計算. setRefLpsの後に行うこと
    def calIcpCovariance(self, estMotion, curScan, cov):
        ratio1, estMotion = self.dass.findCorrespondenceGT(curScan, estMotion)
        # ICPの共分散. ここで得られるのは世界座標系での共分散
        cov = self.cvc.calIcpCovariance(estMotion, self.dass.curLps, self.dass.refLps, cov)
        return cov

    # 逐次SLAM用のセンサ融合. 逐次SLAMでのICPとオドメトリの推定移動量を融合する.
    # dassに参照スキャンを入れておくこと. covに移動量の共分散行列が入る
    def fusePose(self, curScan, estPose, odoMotion, lastPose, fusedPose, fusedCov):
        # ICPの共分散
        # 推定位置estPoseで現在スキャン点群と参照スキャン点群の対応づけ        
        mratio, estPose = self.dass.findCorrespondenceGT(curScan, estPose)
        # ここで得られるのは地図座標系での位置の共分散        
        self.ecov = self.cvc.calIcpCovariance(estPose, self.dass.curLps, self.dass.refLps, self.ecov)

        # オドメトリの位置と共分散
        predPose = Pose2D()  # 予測位置用
        # 直前位置lastPoseに移動量を加えて予測位置を計算        
        predPose = Pose2D.calGlobalPose(odoMotion, lastPose, predPose)
        mcovL = np.zeros((3, 3))
        dT = 0.1 # 0.1(Raspberry Pi Mouse), 0.2(TurtleBot3)
        # オドメトリによる移動量の簡易的な共分散        
        mcovL = self.cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL)
        # 現在位置estPoseで回転させて地図座標系での共分散mcovを得る        
        self.mcov = CovarianceCalculator.rotateCovariance(estPose, mcovL, self.mcov, False)
        # ecov, mcov, covともにlastPoseを原点とした局所座標系での値
        mu1 = np.array([estPose.tx, estPose.ty, DEG2RAD(estPose.th)])  # ICPによる推定値
        mu2 = np.array([predPose.tx, predPose.ty, DEG2RAD(predPose.th)])  # オドメトリによる推定値
        mu = np.empty(3, dtype=float)
        K, mu, fusedCov = self.fuse(mu1, self.ecov, mu2, self.mcov, mu, fusedCov)  # 2つの正規分布の融合
        fusedPose.setVal(mu[0], mu[1], RAD2DEG(mu[2]))  # 融合した移動量を格納
        print("fusedPose: tx=%f ty=%f th=%f" % (fusedPose.tx, fusedPose.ty, fusedPose.th))

        return fusedPose, fusedCov

    def calOdometryCovariance(self, odoMotion, lastPose, mcov):
        mcovL = np.zeros((3, 3))
        dT = 0.1 # 0.1(Raspberry Pi Mouse), 0.2(TurtleBot3)
        mcovL = self.cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL)  # オドメトリで得た移動量の共分散（簡易版）
        mcov = CovarianceCalculator.rotateCovariance(lastPose, mcovL, mcov, False)  # 直前位置lastPoseで回転させて位置の共分散mcovを得る
        return mcov

    # ガウス分布の融合
    # 2つの正規分布を融合する. muは平均, cvは共分散.
    def fuse(self, mu1, cv1, mu2, cv2, mu, cv):
        # 共分散行列の融合
        IC1 = np.linalg.inv(cv1)
        IC2 = np.linalg.inv(cv2)
        IC = IC1 + IC2
        cv = np.linalg.inv(IC)
        # 角度の補正. 融合時に連続性を保つため
        mu11 = mu1  # ICPの方向をオドメトリに合せる
        da = mu2[2] - mu1[2]
        if da > math.pi:
            mu11[2] += 2 * math.pi
        elif da < -math.pi:
            mu11[2] -= 2 * math.pi
        # 平均の融合
        nu1 = np.dot(IC1, mu11)
        nu2 = np.dot(IC2, mu2)
        nu3 = nu1 + nu2
        mu = np.dot(cv, nu3)
        # 角度の補正 (-pi, pi)に収める
        if mu[2] > math.pi:
            mu[2] -= 2 * math.pi
        elif mu[2] < -math.pi:
            mu[2] += 2 * math.pi
        # 係数部の計算
        W1 = np.dot(IC1, mu11)
        W2 = np.dot(IC2, mu2)
        W = np.dot(IC, mu)
        A1 = np.dot(mu1, W1)
        A2 = np.dot(mu2, W2)
        A = np.dot(mu, W)
        K = A1 + A2 - A

        return K, mu, cv
