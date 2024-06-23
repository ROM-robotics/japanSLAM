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

from my_util import DEG2RAD
from l_point2d import ptype


class ScanPointAnalyser:
    # 隣接点との最小距離[m]        
    FPDMIN = 0.06  # これより小さいと誤差が大きくなるので法線計算に使わない. ScanPointResampler.dthrSとずらすこと
    # 隣接点との最大距離[m]    
    FPDMAX = 1.0  #これより大きいと不連続とみなして法線計算に使わない

    def __init__(self, CRTHRE=45, INVALID=-1, costh=math.cos(DEG2RAD(45))):
        self.CRTHRE = CRTHRE
        self.INVALID = INVALID
        self.costh = costh  # 左右の法線方向の食い違いの閾値

    # スキャン点の法線ベクトルを求める. また, 直線, コーナ, 孤立の場合分けをする.
    def analysePoints(self, lps):
        for i in range(len(lps)):
            lp = lps[i]  # スキャン点
            _type = ptype.UNKNOWN
            nL = nR = np.array([0., 0.])
            normal = np.array([0., 0.])
            flagL = self.calNormal(i, lps, -1, nL)  # nLはlpと左側の点で求めた法線ベクトル
            flagR = self.calNormal(i, lps, 1, nR)  # nRはlpと右側の点で求めた法線ベクトル
            nR[0] = -nR[0]  # 符号をnLと合せる
            nR[1] = -nR[1]
            if flagL:
                if flagR:  # 左右両側で法線ベクトルが計算可能
                    if math.fabs(nL[0] * nR[0] + nL[1] * nR[1]) >= self.costh:  # 両側の法線が平行に近い
                        _type = ptype.LINE  # 直線とみなす
                    else:  # 平行から遠ければ,コーナ点とみなす
                        _type = ptype.CORNER
                    # 左右両側の法線ベクトルの平均
                    dx = nL[0] + nR[0]
                    dy = nL[1] + nR[1]
                    L = math.sqrt(dx * dx + dy * dy)
                    if L == 0 :
                        L = 0.000001
                    normal[0] = dx / L
                    normal[1] = dy / L
                else:  # 左側しか法線ベクトルがとれなかった
                    _type = ptype.LINE
                    normal = nL
            else:
                if flagR:  # 右側しか法線ベクトルがとれなかった
                    _type = ptype.LINE
                    normal = nR
                else:  # 両側とも法線ベクトルがとれなかった
                    _type = ptype.ISOLATE  # 孤立点とみなす
                    normal[0] = self.INVALID
                    normal[1] = self.INVALID
            lp.setNormal(normal[0], normal[1])
            lp.setType(_type)

    # 注目点cpの両側の点がcpからdmin以上dmax以下の場合に法線を計算する
    def calNormal(self, idx, lps, direction, normal):
        cp = lps[idx]  # 注目点
        if direction == 1:
            range_end = len(lps)
        elif direction == -1:
            range_end = 0
        else:
            print("ERROR")
        for i in range(idx + direction, range_end, direction):
            if i >= 0:
                lp = lps[i]  # cpのdir（左か右）側の点
                dx = lp.x - cp.x
                dy = lp.y - cp.y
                d = math.sqrt(dx * dx + dy * dy)
                # cpとlpの距離dが適切なら法線計算                
                if d >= ScanPointAnalyser.FPDMIN and d <= ScanPointAnalyser.FPDMAX:
                    normal[0] = dy / d
                    normal[1] = -dx / d
                    return True
                if d > ScanPointAnalyser.FPDMAX:  # どんどん離れるので途中でやめる
                    break
        return False
