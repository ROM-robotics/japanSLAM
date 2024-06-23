#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import PyGnuplot as gp
import numpy as np

from l_point2d import LPoint2D
from pose2d import Pose2D


class MapDrawer:
    def __init__(self, xmin=-10., xmax=10., ymin=-10., ymax=10., aspectR=-1.):
        self.xmin = xmin  # viewer range
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.aspectR = aspectR  # xy ratio

    def setAspectRatio(self, a):
        self.aspectR = a
        gp.c("set size ratio %lf" % (self.aspectR))
        gp.c("set grid")
        gp.c("set tics font ',20'")
#        gp.c("set xrange [-4:4]")
#        gp.c("set yrange [-1.5:1.5]")
        

    # 描画範囲をR四方にする        
    def setRange(self, R):
        self.xmin = self.ymin = -R
        self.xmax = self.ymax = R
        gp.c("set xrange [%lf:%lf]" % (self.xmin, self.xmax))
        gp.c("set yrange [%lf:%lf]" % (self.ymin, self.ymax))

    # 地図と軌跡を描画
    def drawMapGp(self, pcmap):
        self.drawGp(pcmap.globalMap, pcmap.poses)

    # スキャン1個を描画
    def drawScanGp(self, scan):
        poses = np.array([Pose2D()])
        self.drawGp(scan.lps, poses)

    # ロボット軌跡だけを描画
    def drawTrajectoryGp(self, poses):
        lps = np.array([LPoint2D()])
        self.drawGp(lps, poses)

    def drawGp(self, lps, poses):
        gp.c("plot '-' w p pt 7 ps 1.5 lc rgb 0x0, '-' w vector")

        # 点群の描画
        step1 = 1  # 点の間引き間隔.描画が重いとき大きくする
        num = len(lps)
        for i in range(0, num, step1):
            lp = lps[i]
            gp.c("%lf %lf" % (lp.x, lp.y))  # 点の描画
        gp.c("e")

        # ロボット軌跡の描画
        step2 = 10  # ロボット位置の間引き間隔
        num = len(poses)
        for i in range(0, num, step2):
            pose = poses[i]
            cx = pose.tx  # 並進位置
            cy = pose.ty
            cs = pose.Rmat[0, 0]  # 回転角によるcos
            sn = pose.Rmat[1, 0]  # 回転角によるsin

            # ロボット座標系の位置と向きを描く
            dd = 0.4 # 1 for big arrow
            x1 = cs * dd  # ロボット座標系のx軸
            y1 = sn * dd
            x2 = -sn * dd  # ロボット座標系のy軸
            y2 = cs * dd
            gp.c("%lf %lf %lf %lf" % (cx, cy, x1, y1))
            gp.c("%lf %lf %lf %lf" % (cx, cy, x2, y2))
        gp.c("e")
