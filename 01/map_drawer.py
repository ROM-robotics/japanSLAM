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

    # Make the drawing range in R square
    def setRange(self, R):
        self.xmin = self.ymin = -R
        self.xmax = self.ymax = R
        gp.c("set xrange [%lf:%lf]" % (self.xmin, self.xmax))
        gp.c("set yrange [%lf:%lf]" % (self.ymin, self.ymax))

    # Draw a map and trajectory
    def drawMapGp(self, pcmap):
        self.drawGp(pcmap.globalMap, pcmap.poses)

    # Draw one scan
    def drawScanGp(self, scan):
        poses = np.array([Pose2D()])
        self.drawGp(scan.lps, poses)

    # Draw only the robot trajectory
    def drawTrajectoryGp(self, poses):
        lps = np.array([LPoint2D()])
        self.drawGp(lps, poses)

    def drawGp(self, lps, poses):
        gp.c("plot '-' w p pt 7 ps 1.5 lc rgb 0x0, '-' w vector") # Gnuplot settings

        # Drawing the point group
        step1 = 1  # Pen out of the point.Make it bigger when the drawing is heavy
        num = len(lps)
        for i in range(0, num, step1):
            lp = lps[i]
            gp.c("%lf %lf" % (lp.x, lp.y))  # Point drawing
        gp.c("e")

        # Drawing a robot trajectory
        step2 = 10 #10  # Throughout interval of robot position
        num = len(poses)
        for i in range(0, num, step2):
            pose = poses[i]
            cx = pose.tx  # Parallel
            cy = pose.ty
            cs = pose.Rmat[0, 0]  # COS by rotation angle
            sn = pose.Rmat[1, 0]  # SIN by rotation angle

            # Draw the position and direction of the robot coordinate system
            dd = 0.4 # 1 for big arrow
            x1 = cs * dd  # Robot coordinate X -axis
            y1 = sn * dd
            x2 = -sn * dd  # Robot coordinates Y -axis
            y2 = cs * dd
            gp.c("%lf %lf %lf %lf" % (cx, cy, x1, y1))
            gp.c("%lf %lf %lf %lf" % (cx, cy, x2, y2))
        gp.c("e")
