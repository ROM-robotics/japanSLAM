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

    # Based on the data correspondence, Give the initial value INITPOSE and find an estimated Estpose in the robot position
    def optimizePoseSL(self, initPose, estPose):
        th = initPose.th
        tx = initPose.tx
        ty = initPose.ty
        txmin = tx  # Small cost
        tymin = ty
        thmin = th
        evmin = math.inf  # Minimum cost
        evold = evmin  # The previous cost value. Used for convergence judgment
        pose = Pose2D()
        direction = Pose2D()
        ev = self.cfunc.calValuePD(tx, ty, th)  # Cost calculation
        nn = 0  # Repeat number. For confirmation
        while math.fabs(evold - ev) > self.evthre:  # Convergence judgment. Ends if the value changes are small
            nn = nn + 1
            evold = ev
            # Division of numerical calculation
            dx = (self.cfunc.calValuePD(tx + self.dd, ty, th) - ev) / self.dd
            dy = (self.cfunc.calValuePD(tx, ty + self.dd, th) - ev) / self.dd
            dth = (self.cfunc.calValuePD(tx, ty, th + self.da) - ev) / self.da
            tx += dx  # Once the next search position
            ty += dy
            th += dth
            # Straight line search by Brent Law
            pose.tx = tx  # Explore starting point
            pose.ty = ty
            pose.th = th
            direction.tx = dx  # Exploration
            direction.ty = dy
            direction.th = dth
            pose = self.search(ev, pose, direction)  # Straight line search execution
            tx = pose.tx  # The position required for a straight line search
            ty = pose.ty
            th = pose.th
            ev = self.cfunc.calValuePD(tx, ty, th)  # Cost calculation at the required position
            if ev < evmin:  # Updated if the cost is the minimum so far
                evmin = ev
                txmin = tx
                tymin = ty
                thmin = th
        estPose.setVal(txmin, tymin, thmin)  # Save the solution that gives the minimum value
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
