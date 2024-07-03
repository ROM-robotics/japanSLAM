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
        self.curLps = curLps if curLps else np.empty(0)  # 対応がとれた現在スキャンの点群
        self.refLps = refLps if refLps else np.empty(0)  # 対応がとれた参照スキャンの点群
        self.evlimit = evlimit  # マッチングで対応がとれたと見なす距離閾値
        self.pnrate = pnrate  # 誤差がevlimit以内で対応がとれた点の比率

    def setEvlimit(self, e):
        self.evlimit = e

    # DataAssociatorで対応のとれた点群cur, refを設定
    def setPoints(self, cur, ref):
        self.curLps = cur
        self.refLps = ref

    def getPnrate(self):
        return self.pnrate

    # ICP cost function by vertical distance
    def calValuePD(self, tx, ty, th):
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
            # Convert CLP to the coordinate system of the scan
            x = cos_a * cx - sin_a * cy + tx
            y = sin_a * cx + cos_a * cy + ty
            pdis = (x - rlp.x) * rlp.nx + (y - rlp.y) * rlp.ny  # Vertical range
            er = pdis * pdis
            if er <= ev_ev:
                pn += 1  # Number of points with small errors
            error += er  # Cumulative errors at each point
            nn += 1
        error = error / nn if nn > 0 else math.inf  # Average. If the valid score is 0, HUGE_VAL is the value
        self.pnrate = 1.0 * pn / nn if nn > 0 else 0  # nn=Temporary values ​​(error processing)
        error *= 100  # Put 100 so that the valuation value is not too small (the meaning of 100 is thin)
        return error
