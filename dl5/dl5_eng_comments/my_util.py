#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import math


def DEG2RAD(x):
    return (x) * math.pi / 180


def RAD2DEG(x):
    return (x) * 180 / math.pi


class MyUtil:
    @staticmethod
    def add(a1, a2):
        sum = a1 + a2
        if sum < -180:
            sum += 360
        elif sum >= 180:
            sum -= 360
        return sum
