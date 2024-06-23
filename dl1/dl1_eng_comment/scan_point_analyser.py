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
    # The minimum distance from the next point [M]    
    FPDMIN = 0.06  #If it is smaller than this, the error will increase, so it is not used for normal calculation.Slowth with scanpointresampler.dthrs
    # Maximum distance to the next point [M]
    FPDMAX = 1.0  #If it is bigger than this, it is considered discontinuous and do not use it for normal calculation

    def __init__(self, CRTHRE=45, INVALID=-1, costh=math.cos(DEG2RAD(45))):
        self.CRTHRE = CRTHRE
        self.INVALID = INVALID
        self.costh = costh  # The threshold of the left and right normal directions

    # In search of a normal vector for scanning points.Divide straight, corner, isolation.
    def analysePoints(self, lps):
        for i in range(len(lps)):
            lp = lps[i]  # Scanning point
            _type = ptype.UNKNOWN
            nL = nR = np.array([0., 0.])
            normal = np.array([0., 0.])
            flagL = self.calNormal(i, lps, -1, nL)  # nLLP and the Non -vector obtained at the left point
            flagR = self.calNormal(i, lps, 1, nR)  # nRLP and the right side vector obtained at the point on the right side
            nR[0] = -nR[0]  # Combine the sign with NL
            nR[1] = -nR[1]
            if flagL:
                if flagR:  # A normal vector can be calculated on both sides of the left and right
                    if math.fabs(nL[0] * nR[0] + nL[1] * nR[1]) >= self.costh:  # Nothing on both sides is close to parallel
                        _type = ptype.LINE  # Be considered a straight line
                    else:  # If it is far from parallel, consider it as a corner point
                        _type = ptype.CORNER
                    # The average of the normal vector on both sides
                    dx = nL[0] + nR[0]
                    dy = nL[1] + nR[1]
                    L = math.sqrt(dx * dx + dy * dy)
                    if L == 0 :
                        L = 0.000001
                    normal[0] = dx / L
                    normal[1] = dy / L
                else:  # I could only get a normal vector on the left
                    _type = ptype.LINE
                    normal = nL
            else:
                if flagR:  # Only the right side could get a normal vector
                    _type = ptype.LINE
                    normal = nR
                else:  # I couldn't get a normal vector on both sides
                    _type = ptype.ISOLATE  # Think of it as an isolated point
                    normal[0] = self.INVALID
                    normal[1] = self.INVALID
            lp.setNormal(normal[0], normal[1])
            lp.setType(_type)

    # Attention Calculate the normal when both sides of the CP are DMIN or more from CP.
    def calNormal(self, idx, lps, direction, normal):
        cp = lps[idx]  # Notes
        if direction == 1:
            range_end = len(lps)
        elif direction == -1:
            range_end = 0
        else:
            print("ERROR")
        for i in range(idx + direction, range_end, direction):
            if i >= 0:
                lp = lps[i]  # cpDIR (left or right) point
                dx = lp.x - cp.x
                dy = lp.y - cp.y
                d = math.sqrt(dx * dx + dy * dy)
                # cpAnd if the distance D of LP is appropriate, the normal calculation                
                if d >= ScanPointAnalyser.FPDMIN and d <= ScanPointAnalyser.FPDMAX:
                    normal[0] = dy / d
                    normal[1] = -dx / d
                    return True
                if d > ScanPointAnalyser.FPDMAX:  # I'm going away, so stop on the way
                    break
        return False
