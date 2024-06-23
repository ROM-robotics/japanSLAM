#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import math
import scipy.optimize as so

from my_util import MyUtil
from pose2d import Pose2D
from cost_function import CostFunction


class PoseOptimizer:
    def __init__(self, evthre=0.000001, dd=0.00001, da=0.00001, cfunc=None):
        self.evthre = evthre  # コスト変化閾値. 変化量がこれ以下なら繰り返し終了
        self.dd = dd  # 数値微分の刻み（並進）
        self.da = da  # 数値微分の刻み（回転）
        self.cfunc = cfunc if cfunc else CostFunction()  # コスト関数

    def setEvlimit(self, _l):
        self.cfunc.setEvlimit(_l)

    def setPoints(self, curLps, refLps):
        self.cfunc.setPoints(curLps, refLps)

    def setEvthre(self, inthre):
        self.evthre = inthre

    def getPnrate(self):
        return self.cfunc.getPnrate()

    # データ対応づけ固定のもと, 初期値initPoseを与えてロボット位置の推定値estPoseを求める
    def optimizePoseSL(self, initPose, estPose):
        th = initPose.th
        tx = initPose.tx
        ty = initPose.ty
        txmin = tx  # コスト最小の解
        tymin = ty
        thmin = th
        evmin = math.inf  # コストの最小値
        evold = evmin  # 1つ前のコスト値. 収束判定に使う
        pose = Pose2D()
        direction = Pose2D()
        ev = self.cfunc.calValuePD(tx, ty, th)  # コスト計算
        nn = 0  # 繰り返し回数. 確認用
        while math.fabs(evold - ev) > self.evthre:  # 収束判定. 値の変化が小さいと終了
            nn = nn + 1
            evold = ev
            # 数値計算による偏微分
            dx = (self.cfunc.calValuePD(tx + self.dd, ty, th) - ev) / self.dd
            dy = (self.cfunc.calValuePD(tx, ty + self.dd, th) - ev) / self.dd
            dth = (self.cfunc.calValuePD(tx, ty, th + self.da) - ev) / self.da
            tx += dx  # いったん次の探索位置を決める
            ty += dy
            th += dth
            # ブレント法による直線探索
            pose.tx = tx  # 探索開始点
            pose.ty = ty
            pose.th = th
            direction.tx = dx  # 探索方向
            direction.ty = dy
            direction.th = dth
            pose = self.search(ev, pose, direction)  # 直線探索実行
            tx = pose.tx  # 直線探索で求めた位置
            ty = pose.ty
            th = pose.th
            ev = self.cfunc.calValuePD(tx, ty, th)  # 求めた位置でコスト計算
            if ev < evmin:  # コストがこれまでの最小なら更新
                evmin = ev
                txmin = tx
                tymin = ty
                thmin = th
        estPose.setVal(txmin, tymin, thmin)  # 最小値を与える解を保存
        return evmin, estPose

    # Line search ブレント法で直線探索を行う
    # poseを始点に, dp方向にどれだけ進めばよいかステップ幅を見つける.
    def search(self, ev0, pose, dp):
        result = so.fminbound(self.objFunc, -2.0, 2.0, args=(pose, dp), full_output=1)  # 探索範囲(-2.0,2.0), 経験的な最大繰り返し回数
        t = result[0]  # 求めるステップ幅
        pose.tx = pose.tx + t * dp.tx  # 求める最小解をposeに格納
        pose.ty = pose.ty + t * dp.ty
        pose.th = MyUtil.add(pose.th, t * dp.th)
        return pose

    # 直線探索の目的関数. ttがステップ幅
    def objFunc(self, tt, pose, dp):
        tx = pose.tx + tt * dp.tx  # poseからdp方向にttだけ進む
        ty = pose.ty + tt * dp.ty
        th = MyUtil.add(pose.th, tt * dp.th)
        v = self.cfunc.calValuePD(tx, ty, th)  # コスト関数値
        return v
