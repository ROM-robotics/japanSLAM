#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import math
import scipy.optimize as so

from my_util import MyUtil
from pose2d import Pose2D
from cost_function import CostFunction


class PoseOptimizer:
    def __init__(self, evthre=0.000001, dd=0.00001, da=0.00001, cfunc=None):        
        self.evthre = evthre  # Cost change threshold.
        self.dd = dd  # Numeric differentiation (parallel)
        self.da = da  # Numeric differentiation (rotation)
        self.cfunc = cfunc if cfunc else CostFunction()  # Cost function

    def setEvlimit(self, _l):
        self.cfunc.setEvlimit(_l)

    def setPoints(self, curLps, refLps):
        self.cfunc.setPoints(curLps, refLps)

    def setEvthre(self, inthre):
        self.evthre = inthre

    def getPnrate(self):
        return self.cfunc.getPnrate()

    # Under the fixation of data, give the initial value INITPOSE and find the estimated Estpose for the robot position.
    def optimizePoseSL(self, initPose, estPose): # (လိုင်းနံပါတ် ၃၈ )
        th = initPose.th
        tx = initPose.tx
        ty = initPose.ty
        txmin = tx  # Small cost
        tymin = ty
        thmin = th
        evmin = math.inf  # Minimum cost
        evold = evmin  # The previous cost value. Used for convergence judgment
        pose = Pose2D()
        direction = Pose2D()
        ev = self.cfunc.calValuePD(tx, ty, th)  # Cost calculation # (လိုင်းနံပါတ် ၄၉ )
        nn = 0  # Repeated number. For confirmation
        while math.fabs(evold - ev) > self.evthre:  # Convergence judgment. Ends if the value changes is small
            nn = nn + 1
            evold = ev
            # Division of numerical calculation
            dx = (self.cfunc.calValuePD(tx + self.dd, ty, th) - ev) / self.dd
            dy = (self.cfunc.calValuePD(tx, ty + self.dd, th) - ev) / self.dd
            dth = (self.cfunc.calValuePD(tx, ty, th + self.da) - ev) / self.da
            tx += dx  # Once the next search position
            ty += dy
            th += dth
            # Straight line search by Brent Law
            pose.tx = tx  # Explore starting point
            pose.ty = ty
            pose.th = th
            direction.tx = dx  # Exploration
            direction.ty = dy
            direction.th = dth
            pose = self.search(ev, pose, direction)  # Straight line search execution
            tx = pose.tx  # The position required for a straight line search
            ty = pose.ty
            th = pose.th
            ev = self.cfunc.calValuePD(tx, ty, th)  # Cost calculation at the required position # (လိုင်းနံပါတ် ၇၂ )
            if ev < evmin:  # Updated if the cost is the minimum so far
                evmin = ev
                txmin = tx
                tymin = ty
                thmin = th
        estPose.setVal(txmin, tymin, thmin)  # Save the solution that gives the minimum value
        return evmin, estPose # # (လိုင်းနံပါတ် ၇၉ )

    # Execute straight exploration by LINE SEARCH Brent Law
    # Find the step width of how to proceed in the DP direction with Pose.
    def search(self, ev0, pose, dp):
        result = so.fminbound(self.objFunc, -2.0, 2.0, args=(pose, dp), full_output=1)  # Exploration range (-2.0, 2.0), maximum number of experienced repetitions
        t = result[0]  # The step width to be asked for
        pose.tx = pose.tx + t * dp.tx  # Store the smallest you want in Pose
        pose.ty = pose.ty + t * dp.ty
        pose.th = MyUtil.add(pose.th, t * dp.th)
        return pose

    # Purpose function of straight line search. TT is step width
    def objFunc(self, tt, pose, dp):
        tx = pose.tx + tt * dp.tx  # Proceed only TT in the direction of DP from Pose
        ty = pose.ty + tt * dp.ty
        th = MyUtil.add(pose.th, tt * dp.th)
        v = self.cfunc.calValuePD(tx, ty, th)  # Cost value
        return v
