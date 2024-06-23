#!/usr/bin/python
# coding: utf-8
import numpy as np
import math

from my_util import DEG2RAD, RAD2DEG, MyUtil
from l_point2d import ptype

# Calculate the specification of estimated values ​​by ICP and the co -spoding of the estimated values ​​by odmetry.
class CovarianceCalculator:
    def __init__(self, dd=0.00001, da=0.00001, a1=0., a2=0.):
        self.dd = dd
        self.da = da
        self.a1 = a1
        self.a2 = a2

    def setAlpha(self, a1, a2):
        self.a1 = a1
        self.a2 = a2

    # In search of co -diversification COV of the estimated values ​​of the robot position by ICP
    # Estimated position Pose, currently scanning group CURLPS, reference scanning group Reflps
    def calIcpCovariance(self, pose, curLps, refLps, cov):
        tx = pose.tx
        ty = pose.ty
        th = pose.th
        a = DEG2RAD(th)
        Jx_list = list()
        Jy_list = list()
        Jt_list = list()

        for i in range(len(curLps)):
            clp = curLps[i]  # Currently scanning points
            rlp = refLps[i]  # Points of reference scanning
            if rlp.type == ptype.ISOLATE:  # Excludes isolation points
                continue
            pd0 = self.calPDistance(clp, rlp, tx, ty, a)  # Cost value
            pdx = self.calPDistance(clp, rlp, tx + self.dd, ty, a)  # Cost function value that changed X slightly
            pdy = self.calPDistance(clp, rlp, tx, ty + self.dd, a)  # Cost function value that changed Y slightly
            pdt = self.calPDistance(clp, rlp, tx, ty, a + self.da)  # Cost function value that changed TH slightly

            Jx_list.append((pdx - pd0) / self.dd)  # Partial slightly differential (X ingredients)
            Jy_list.append((pdy - pd0) / self.dd)  # Unevenness (Y component)
            Jt_list.append((pdt - pd0) / self.da)  # Polymodes (TH ingredients)

        # Calculation of near J^TJ of the Hesse matrix
        hes = np.zeros((3, 3))  # Appearance Hesse matrix.Initialize at 0
        for i in range(len(Jx_list)):
            hes[0, 0] += Jx_list[i] * Jx_list[i]
            hes[0, 1] += Jx_list[i] * Jy_list[i]
            hes[0, 2] += Jx_list[i] * Jt_list[i]
            hes[1, 1] += Jy_list[i] * Jy_list[i]
            hes[1, 2] += Jy_list[i] * Jt_list[i]
            hes[2, 2] += Jt_list[i] * Jt_list[i]

        # Use J^TJ is a symmetric matrix
        hes[1, 0] = hes[0, 1]
        hes[2, 0] = hes[0, 2]
        hes[2, 1] = hes[1, 2]

        cov = np.linalg.inv(hes)  # The coordinated matrix is ​​inverted in the (approximation) Hesse matrix

        return cov

    # Observation model formula using vertical distance
    def calPDistance(self, clp, rlp, tx, ty, th):
        x = math.cos(th) * clp.x - math.sin(th) * clp.y + tx  # Coordinate conversion of CLP at an estimated position
        y = math.sin(th) * clp.x + math.cos(th) * clp.y + ty
        pdis = (x - rlp.x) * rlp.nx + (y - rlp.y) * rlp.ny  # Valley range from coordinate conversion to RLP
        return pdis

    # Co -diversification of estimated values ​​by odometry
    def calMotionCovarianceSimple(self, motion, dT, cov):
        dis = math.sqrt(motion.tx * motion.tx + motion.ty * motion.ty)  # Moving distance
        vt = dis / dT  # Putting speed [m/s]
        wt = DEG2RAD(motion.th) / dT  # Corner speed [RAD/S]
        vthre = 0.001  # The lower limit of VT. Includes measures when 0 is 0 due to synchronization
        wthre = 0.01  # WT lower limit0.05(Raspberry Pi Mouse), 0.01(TurtleBot3)

        if vt < vthre:
            vt = vthre
        if wt < wthre:
            wt = wthre

        dx = vt
        dy = vt
        da = wt

        C1 = np.eye(3)
        C1[0, 0] = 1. * dx * dx  # Put in ingredient x 3.0(Raspberry Pi Mouse), 1.0(TurtleBot3)
        C1[1, 1] = 1. * dy * dy  # Altitude ingredient Y 3.0(Raspberry Pi Mouse), 1.0(TurtleBot3)
        C1[2, 2] = 25. * da * da  # When the rotation component of the rotating component is large 300.0(Raspberry Pi Mouse), 25.0(TurtleBot3)  

        cov = C1

        return cov

    # Rotate the coordinated matrix COV by the angle of Pose
    @staticmethod
    def rotateCovariance(pose, cov, icov, reverse):
        cs = math.cos(DEG2RAD(pose.th))  # COS by Pose's rotating ingredient TH
        sn = math.sin(DEG2RAD(pose.th))
        J = np.zeros((3, 3))  # Rotation jakobi matrix
        J[0, 0] = cs
        J[0, 1] = -sn
        J[0, 2] = 0.
        J[1, 0] = sn
        J[1, 1] = cs
        J[1, 2] = 0.
        J[2, 0] = 0.
        J[2, 1] = 0.
        J[2, 2] = 1.
        JT = J.transpose()
        if reverse:
            icov = JT @ cov @ J  # Reverse conversion
        else:
            icov = J @ cov @ JT  # Rotation transformation
        return icov
