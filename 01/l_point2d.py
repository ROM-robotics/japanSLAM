#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

from enum import Enum
import math

from my_util import DEG2RAD


class ptype(Enum):
    UNKNOWN = 0
    LINE = 1
    CORNER = 2
    ISOLATE = 3


class LPoint2D:
    def __init__(self, sid=-1, x=0., y=0., nx=0., ny=0., atd=0.):
        self.sid = int(sid)
        self.x = x
        self.y = y
        self.nx = nx
        self.ny = ny
        self.atd = atd
        self.type = ptype.UNKNOWN

    def setData(self, sid, x, y):
        self.sid = sid
        self.x = x
        self.y = y

    # rangeとangleからxyを求める(右手系)
    def calXY(self, _range, angle):
        a = DEG2RAD(angle)
        self.x = _range * math.cos(a)
        self.y = _range * math.sin(a)

    def setSid(self, sid):
        self.sid = int(sid)

    def setType(self, t):
        self.type = t

    def setNormal(self, nx, ny):
        self.nx = nx
        self.ny = ny
