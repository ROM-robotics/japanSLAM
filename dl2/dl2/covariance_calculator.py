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

from my_util import DEG2RAD, RAD2DEG, MyUtil
from l_point2d import ptype

# ICPによる推定値の共分散, および, オドメトリによる推定値の共分散を計算する.
class CovarianceCalculator:
    def __init__(self, dd=0.00001, da=0.00001, a1=0., a2=0.):
        self.dd = dd
        self.da = da
        self.a1 = a1
        self.a2 = a2

    def setAlpha(self, a1, a2):
        self.a1 = a1
        self.a2 = a2

    # ICPによるロボット位置の推定値の共分散covを求める
    # 推定位置pose, 現在スキャン点群curLps, 参照スキャン点群refLps
    def calIcpCovariance(self, pose, curLps, refLps, cov):
        tx = pose.tx
        ty = pose.ty
        th = pose.th
        a = DEG2RAD(th)
        Jx_list = list()
        Jy_list = list()
        Jt_list = list()

        for i in range(len(curLps)):
            clp = curLps[i]  # 現在スキャンの点
            rlp = refLps[i]  # 参照スキャンの点
            if rlp.type == ptype.ISOLATE:  # 孤立点は除外
                continue
            pd0 = self.calPDistance(clp, rlp, tx, ty, a)  # コスト関数値
            pdx = self.calPDistance(clp, rlp, tx + self.dd, ty, a)  # xを少し変えたコスト関数値
            pdy = self.calPDistance(clp, rlp, tx, ty + self.dd, a)  # yを少し変えたコスト関数値
            pdt = self.calPDistance(clp, rlp, tx, ty, a + self.da)  # thを少し変えたコスト関数値

            Jx_list.append((pdx - pd0) / self.dd)  # 偏微分（x成分）
            Jy_list.append((pdy - pd0) / self.dd)  # 偏微分（y成分）
            Jt_list.append((pdt - pd0) / self.da)  # 偏微分（th成分）

        # ヘッセ行列の近似J^TJの計算
        hes = np.zeros((3, 3))  # 近似ヘッセ行列。0で初期化
        for i in range(len(Jx_list)):
            hes[0, 0] += Jx_list[i] * Jx_list[i]
            hes[0, 1] += Jx_list[i] * Jy_list[i]
            hes[0, 2] += Jx_list[i] * Jt_list[i]
            hes[1, 1] += Jy_list[i] * Jy_list[i]
            hes[1, 2] += Jy_list[i] * Jt_list[i]
            hes[2, 2] += Jt_list[i] * Jt_list[i]

        # J^TJが対称行列であることを利用
        hes[1, 0] = hes[0, 1]
        hes[2, 0] = hes[0, 2]
        hes[2, 1] = hes[1, 2]

        cov = np.linalg.inv(hes)  # 共分散行列は（近似）ヘッセ行列の逆行列

        return cov

    # 垂直距離を用いた観測モデルの式
    def calPDistance(self, clp, rlp, tx, ty, th):
        x = math.cos(th) * clp.x - math.sin(th) * clp.y + tx  # clpを推定位置で座標変換
        y = math.sin(th) * clp.x + math.cos(th) * clp.y + ty
        pdis = (x - rlp.x) * rlp.nx + (y - rlp.y) * rlp.ny  # 座標変換した点からrlpへの垂直距離
        return pdis

    # オドメトリによる推定値の共分散    
    def calMotionCovarianceSimple(self, motion, dT, cov):
        dis = math.sqrt(motion.tx * motion.tx + motion.ty * motion.ty)  # 移動距離
        vt = dis / dT  # 並進速度[m/s]
        wt = DEG2RAD(motion.th) / dT  # 角速度[rad/s]
        vthre = 0.001  # vtの下限値. 同期ずれで0になる場合の対処も含む
        wthre = 0.05  # wtの下限値 0.05(Raspberry Pi Mouse), 0.01(TurtleBot3)

        if vt < vthre:
            vt = vthre
        if wt < wthre:
            wt = wthre

        dx = vt
        dy = vt
        da = wt

        C1 = np.eye(3)
        C1[0, 0] = 3. * dx * dx  # 並進成分x 3.0(Raspberry Pi Mouse), 1.0(TurtleBot3)
        C1[1, 1] = 3. * dy * dy  # 並進成分y 3.0(Raspberry Pi Mouse), 1.0(TurtleBot3)
        C1[2, 2] =300. * da * da  # 回転成分 オドメトリの回転成分ずれが大きい場合 300.0(Raspberry Pi Mouse), 25.0(TurtleBot3)  

        cov = C1

        return cov

    # 共分散行列covをposeの角度分だけ回転させる
    @staticmethod
    def rotateCovariance(pose, cov, icov, reverse):
        cs = math.cos(DEG2RAD(pose.th))  # poseの回転成分thによるcos
        sn = math.sin(DEG2RAD(pose.th))
        J = np.zeros((3, 3))  # 回転のヤコビ行列
        J[0, 0] = cs
        J[0, 1] = -sn
        J[0, 2] = 0.
        J[1, 0] = sn
        J[1, 1] = cs
        J[1, 2] = 0.
        J[2, 0] = 0.
        J[2, 1] = 0.
        J[2, 2] = 1.
        JT = J.transpose()
        if reverse:
            icov = JT @ cov @ J  # 逆回転変換
        else:
            icov = J @ cov @ JT  # 回転変換
        return icov
