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

from my_util import RAD2DEG, DEG2RAD
from pose2d import Pose2D
from data_associator import DataAssociator
from covariance_calculator import CovarianceCalculator


# Sensor fusioner. Combine the estimated values ​​of ICP and odometri
class PoseFuser:
    def __init__(self, ecov=None, mcov=None, totalCov=None, dass=None, cvc=None):
        self.ecov = ecov if ecov else np.zeros((3, 3))  # ICP coordinated matrix
        self.mcov = mcov if mcov else np.zeros((3, 3))  # Odometry coordinated matrix
        self.totalCov = totalCov if totalCov else np.zeros((3, 3))
        self.dass = dass if dass else DataAssociator()  # Data -compatible device
        self.cvc = cvc if cvc else CovarianceCalculator()  # Calculator

    def setRefScan(self, refScan):
        self.dass.setRefBaseGT(refScan.lps)

    # Sensor fusion for one -off SLAM. Combine the estimated amount of mobility between ICP and odometry in SLAM..
    # Put the reference scan in the dass. There is a coordinated matrix with the amount of movement in the COV
    def fusePose(self, curScan, estPose, odoMotion, lastPose, fusedPose, fusedCov):
        # ICP coordination
        # Current scanning group and reference scanning group with estimated position Estpose        
        mratio, estPose = self.dass.findCorrespondenceGT(curScan, estPose)
        # What can be obtained here is the coordination of the position in the map coordinate system.        
        self.ecov = self.cvc.calIcpCovariance(estPose, self.dass.curLps, self.dass.refLps, self.ecov)

        # Distributed with the position of the odometri
        predPose = Pose2D()  # For predictive position
        # Calculate the predicted position by adding the moving amount to the last position Lastpose        
        predPose = Pose2D.calGlobalPose(odoMotion, lastPose, predPose)
        mcovL = np.zeros((3, 3))
        dT = 0.1 # 0.1(Raspberry Pi Mouse), 0.2(TurtleBot3)
        # Simple coordination of the amount of movement by odmetry
        mcovL = self.cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL)

        # Rotate the current location ESTPose to obtain a coordinated MCOV in the map coordinate system        
        self.mcov = CovarianceCalculator.rotateCovariance(estPose, mcovL, self.mcov, False)
        # ecov, mcov, Both COV value in the local coordinate system based on LastPose
        mu1 = np.array([estPose.tx, estPose.ty, DEG2RAD(estPose.th)])  # Estimated value by ICP
        mu2 = np.array([predPose.tx, predPose.ty, DEG2RAD(predPose.th)])  # Estimated value by odmetry
        mu = np.empty(3, dtype=float)
        mu, fusedCov = self.fuse(mu1, self.ecov, mu2, self.mcov, mu, fusedCov)  # Fusion of two normal distribution        
        fusedPose.setVal(mu[0], mu[1], RAD2DEG(mu[2]))  # Stores the fused amount
        print("fusedPose: tx=%f ty=%f th=%f" % (fusedPose.tx, fusedPose.ty, fusedPose.th))

        return fusedPose, fusedCov

    # Gaussian distribution fusion
    # Combine two normal distributions. MU is average, CV is diversified.
    def fuse(self, mu1, cv1, mu2, cv2, mu, cv):
        # Fusion of coordinates
        IC1 = np.linalg.inv(cv1)
        IC2 = np.linalg.inv(cv2)
        IC = IC1 + IC2
        cv = np.linalg.inv(IC)
        # Angle correction. To maintain continuity during fusion
        mu11 = mu1  # Complete the direction of the ICP to the odometri
        da = mu2[2] - mu1[2]
        if da > math.pi:
            mu11[2] += 2 * math.pi
        elif da < -math.pi:
            mu11[2] -= 2 * math.pi
        # Average fusion
        nu1 = np.dot(IC1, mu11)
        nu2 = np.dot(IC2, mu2)
        nu3 = nu1 + nu2
        mu = np.dot(cv, nu3)
        # Angle correction (-pi, pi)Put in
        if mu[2] > math.pi:
            mu[2] -= 2 * math.pi
        elif mu[2] < -math.pi:
            mu[2] += 2 * math.pi

        return mu, cv
