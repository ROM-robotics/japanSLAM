#!/usr/bin/python
# coding: utf-8

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

    # Convert point p in a global coordinate system to your (Pose2d) local coordinate system
    def relativePoint(self, l_point2d):
        dx = l_point2d.x - self.tx
        dy = l_point2d.y - self.ty
        x = dx * self.Rmat[0, 0] + dy * self.Rmat[1, 0]  # Inverse matrix of rotation
        y = dx * self.Rmat[0, 1] + dy * self.Rmat[1, 1]
        return LPoint2D(l_point2d.sid, x, y)

    # Convert the point p in the local coordinate system of yourself (Pose2d) into a global coordinate system
    def globalPoint(self, l_point2d):
        x = self.Rmat[0, 0] * l_point2d.x + self.Rmat[0, 1] * l_point2d.y + self.tx
        y = self.Rmat[1, 0] * l_point2d.x + self.Rmat[1, 1] * l_point2d.y + self.ty
        return LPoint2D(l_point2d.sid, x, y)

    # Convert the point P in the local coordinate system to a global coordinate system and put it into PO.
    def globalPoint_io(self, l_point2d_i, l_point2d_o):
        l_point2d_o.x = self.Rmat[0, 0] * l_point2d_i.x + self.Rmat[0, 1] * l_point2d_i.y + self.tx
        l_point2d_o.y = self.Rmat[1, 0] * l_point2d_i.x + self.Rmat[1, 1] * l_point2d_i.y + self.ty

    # Relative position of the current coordinate system npose as viewed from the reference coordinates BPOSE Relpose（Inverse compounding operator）
    # The amount moved between T-1 and T is expressed in the coordinate system of T-1    
    def calRelativePose(self, b_pose, rel_pose):
        # Tongue
        dx = self.tx - b_pose.tx
        dy = self.ty - b_pose.ty
        rel_pose.tx = b_pose.Rmat[0][0] * dx + b_pose.Rmat[1][0] * dy
        rel_pose.ty = b_pose.Rmat[0][1] * dx + b_pose.Rmat[1][1] * dy

        # rotate
        th = self.th - b_pose.th
        if th < -180:
            th += 360
        elif th >= 180:
            th -= 360
        rel_pose.th = th
        rel_pose.calRmat()
        return rel_pose

    # Find a coordinate system NPose that advanced only from the relative position from the reference coordinates BPOSE (Compounding operator）
    @staticmethod
    def calGlobalPose(rel_pose, b_pose, n_pose):
        # Tongue
        tx = rel_pose.tx
        ty = rel_pose.ty
        
        n_pose.tx = b_pose.Rmat[0][0] * tx + b_pose.Rmat[0][1] * ty + b_pose.tx
        n_pose.ty = b_pose.Rmat[1][0] * tx + b_pose.Rmat[1][1] * ty + b_pose.ty

        # angle
        th = b_pose.th + rel_pose.th
        if th < -180:
            th += 360
        elif th >= 180:
            th -= 360
        n_pose.th = th
        n_pose.calRmat()

        return n_pose


class PoseCov:
    def __init__(self, pose=Pose2D(), cov=np.array([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]])):
        self.pose = pose
        self.cov = cov
