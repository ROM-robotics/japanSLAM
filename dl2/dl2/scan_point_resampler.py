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

from l_point2d import LPoint2D


class ScanPointResampler:
    def __init__(self, dthreS=0.05, dthreL=0.25, dis=0.):
        self.dthreS = dthreS  # 点の距離間隔[m]
        self.dthreL = dthreL  # 点の距離閾値[m]. この間隔を超えたら補間しない
        self.dis = dis  # 累積距離, 作業用
        self.inserted = False  # cpより前にnnpを入れたというフラグ

    def resamplePoints(self, scan):
        lps = scan.lps  # スキャン点群
        num = len(lps)
        if num == 0:
            return
        newLps = np.empty(0)  # リサンプル後の点群
        lp = lps[0]
        prevLp = lp
        nnp = LPoint2D(lp.sid, lp.x, lp.y)
        newLps = np.append(newLps, nnp)  # 最初の点は入れる
        newLps_list = newLps.tolist()
        for i in range(1, num):
            lp = lps[i]  # スキャン点
            self.inserted = False
            exist = self.findInterpolatePoint(lp, prevLp, nnp)
            if exist:  # 入れる点がある
                newLps_list.append(copy.deepcopy(nnp))  # 新しい点nnpを入れる
                prevLp = nnp  # nnpが直前点になる
                # lpの前で補間点を入れたので、lpをもう一度やる
                if self.inserted:
                    i = i - 1
            else:
                prevLp = lp  # 今のlpが直前点になる
        newLps = np.asarray(newLps_list)
        scan.setLps(newLps)

    def findInterpolatePoint(self, cp, pp, nnp):
        dx = cp.x - pp.x
        dy = cp.y - pp.y
        L = math.sqrt(dx * dx + dy * dy)  # 現在点cpと直前点ppの距離
        if L == 0 :
            L = 0.000001
        if self.dis + L < self.dthreS:  # 予測累積距離(dis+L)がdthreSより小さい点は削除
            self.dis = self.dis + L  # disに加算
            return False
        elif self.dis + L >= self.dthreL:  # 予測累積距離がdthreLより大きい点は補間せずそのまま残す
            nnp.setData(cp.sid, cp.x, cp.y)
        else:  # 予測累積距離がdthreSを超えたらdthreSになるように補間する
            ratio = (self.dthreS - self.dis) / L
            x2 = dx * ratio + pp.x  # 少し伸ばして距離がdthreSになる位置
            y2 = dy * ratio + pp.y
            nnp.setData(cp.sid, x2, y2)
            self.inserted = True  # cpより前にnnpを入れたというフラグ
        return True
