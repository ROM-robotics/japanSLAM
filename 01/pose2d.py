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
from l_point2d import LPoint2D


class Pose2D:
    def __init__(self, tx=0., ty=0., th=0.):
        self.tx = tx
        self.ty = ty
        self.th = th
        self.Rmat = np.eye(2)
        self.calRmat()

    def setVal(self, x, y, a):
        self.tx = x
        self.ty = y
        self.th = a
        self.calRmat()

    def calRmat(self):
        a = DEG2RAD(self.th)
        self.Rmat[0, 0] = self.Rmat[1, 1] = math.cos(a)
        self.Rmat[1, 0] = math.sin(a)
        self.Rmat[0, 1] = -self.Rmat[1, 0]

    def setAngle(self, th):
        self.th = th

    # グローバル座標系での点pを, 自分（Pose2D）の局所座標系に変換
    def relativePoint(self, l_point2d):
        dx = l_point2d.x - self.tx
        dy = l_point2d.y - self.ty
        x = dx * self.Rmat[0, 0] + dy * self.Rmat[1, 0]  # 回転の逆行列
        y = dx * self.Rmat[0, 1] + dy * self.Rmat[1, 1]
        return LPoint2D(l_point2d.sid, x, y)

    # 自分（Pose2D）の局所座標系での点pをグローバル座標系に変換
    def globalPoint(self, l_point2d):
        x = self.Rmat[0, 0] * l_point2d.x + self.Rmat[0, 1] * l_point2d.y + self.tx
        y = self.Rmat[1, 0] * l_point2d.x + self.Rmat[1, 1] * l_point2d.y + self.ty
        return LPoint2D(l_point2d.sid, x, y)

    # 自分（Pose2D）の局所座標系での点pをグローバル座標系に変換してpoに入れる
    def globalPoint_io(self, l_point2d_i, l_point2d_o):
        l_point2d_o.x = self.Rmat[0, 0] * l_point2d_i.x + self.Rmat[0, 1] * l_point2d_i.y + self.tx
        l_point2d_o.y = self.Rmat[1, 0] * l_point2d_i.x + self.Rmat[1, 1] * l_point2d_i.y + self.ty

    # 基準座標系bposeから見た現座標系nposeの相対位置relPoseを求める（Inverse compounding operator）
    # t-1からtの間に移動した量をt-1の座標系で表す
    def calRelativePose(self, b_pose, rel_pose):
        # 並進
        dx = self.tx - b_pose.tx
        dy = self.ty - b_pose.ty
        rel_pose.tx = b_pose.Rmat[0][0] * dx + b_pose.Rmat[1][0] * dy
        rel_pose.ty = b_pose.Rmat[0][1] * dx + b_pose.Rmat[1][1] * dy

        # 回転
        th = self.th - b_pose.th
        if th < -180:
            th += 360
        elif th >= 180:
            th -= 360
        rel_pose.th = th
        rel_pose.calRmat()
        return rel_pose

    # 基準座標系bposeから相対位置relPoseだけ進んだ座標系nposeを求める（Compounding operator）
    @staticmethod
    def calGlobalPose(rel_pose, b_pose, n_pose):
        # 並進
        tx = rel_pose.tx
        ty = rel_pose.ty
        n_pose.tx = b_pose.Rmat[0][0] * tx + b_pose.Rmat[0][1] * ty + b_pose.tx
        n_pose.ty = b_pose.Rmat[1][0] * tx + b_pose.Rmat[1][1] * ty + b_pose.ty

        # 角度
        th = b_pose.th + rel_pose.th
        if th < -180:
            th += 360
        elif th >= 180:
            th -= 360
        n_pose.th = th
        n_pose.calRmat()

        return n_pose
