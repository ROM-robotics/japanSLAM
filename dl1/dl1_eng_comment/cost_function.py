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


class CostFunction:
    def __init__(self, curLps=None, refLps=None, evlimit=0., pnrate=0.):
        self.curLps = curLps if curLps else np.empty(0)  # Current scanning group of scan
        self.refLps = refLps if refLps else np.empty(0)  # A group of reference scans that have been corresponded
        self.evlimit = evlimit  # Distance threshold that seems to have been taken by matching
        self.pnrate = pnrate  # The ratio of the fact that the error is within EVLIMIT

    def setEvlimit(self, e):
        self.evlimit = e

    # DataAssociatorSet the corresponding point CUR and REF
    def setPoints(self, cur, ref):
        self.curLps = cur
        self.refLps = ref

    def getPnrate(self):
        return self.pnrate

    # Cost function of ICP due to vertical distance
    def calValuePD(self, tx, ty, th):       # ( လိုင်းနံပါတ် ၃၆ )
        a = DEG2RAD(th)
        cos_a = math.cos(a)
        sin_a = math.sin(a)
        error = 0.
        pn = 0
        nn = 0
        line = ptype.LINE
        ev_ev = self.evlimit * self.evlimit

        for clp, rlp in zip(self.curLps, self.refLps):
            if rlp.type != line:  # Do not use unless it is on a straight line
                continue
            cx, cy = clp.x, clp.y
            # clpConvert to reference scan coordinate system
            x = cos_a * cx - sin_a * cy + tx
            y = sin_a * cx + cos_a * cy + ty
            pdis = (x - rlp.x) * rlp.nx + (y - rlp.y) * rlp.ny  # Vertical range
            er = pdis * pdis
            if er <= ev_ev:
                pn += 1  #Number of points with small errors
            error += er  # Cumulative errors at each point
            nn += 1
        error = error / nn if nn > 0 else math.inf  # Take the average. If the valid score is 0, the value isHUGE_VAL
        self.pnrate = 1.0 * pn / nn if nn > 0 else 0  # nn=0I want to make sure that
        error *= 100  # Put 100 so that the valuation value is not too small (the meaning of 100 is thin)
        return error
