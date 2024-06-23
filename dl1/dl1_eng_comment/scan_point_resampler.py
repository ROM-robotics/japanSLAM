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
        self.dthreS = dthreS  # Distance interval [M]
        self.dthreL = dthreL  # Distance threshold of point [M]. Do not interpolate if this interval is exceeded
        self.dis = dis  # Cumulative distance, for work
        self.inserted = False  # cpFlag that put NNP before

    def resamplePoints(self, scan):
        lps = scan.lps  # Scan points
        num = len(lps)
        if num == 0:
            return
        newLps = np.empty(0)  # For point groups after recruitment
        lp = lps[0]
        prevLp = lp
        nnp = LPoint2D(lp.sid, lp.x, lp.y)
        newLps = np.append(newLps, nnp)  # Include the first point
        newLps_list = newLps.tolist()
        for i in range(1, num):
            lp = lps[i]  # Scanning point
            self.inserted = False
            exist = self.findInterpolatePoint(lp, prevLp, nnp)
            if exist:  # There is a point to enter
                newLps_list.append(copy.deepcopy(nnp))  # Put a new point NNP
                prevLp = nnp  # nnpIs just before
                # lpI put an interpotation in front of me, so I will do the LP again
                if self.inserted:
                    i = i - 1
            else:
                prevLp = lp  # The current LP will be just before
        newLps = np.asarray(newLps_list)
        scan.setLps(newLps)

    def findInterpolatePoint(self, cp, pp, nnp):
        dx = cp.x - pp.x
        dy = cp.y - pp.y
        L = math.sqrt(dx * dx + dy * dy)  # Currently point CP and just before PP
        if L == 0 :
            L = 0.000001
        if self.dis + L < self.dthreS:  # Deleted points that are smaller than Dthres
            self.dis = self.dis + L  # disAdded to
            return False
        elif self.dis + L >= self.dthreL:  # Leave the point that the prediction cumulative distance is larger than the dthrel as it is.
            nnp.setData(cp.sid, cp.x, cp.y)
        else:  # When the prediction cumulative distance exceeds Dthres, interpolate to Dthres
            ratio = (self.dthreS - self.dis) / L
            x2 = dx * ratio + pp.x  # A position where the distance is a little extended and the distance becomes DTHRES
            y2 = dy * ratio + pp.y
            nnp.setData(cp.sid, x2, y2)
            self.inserted = True  # cpFlag that put NNP before
        return True
