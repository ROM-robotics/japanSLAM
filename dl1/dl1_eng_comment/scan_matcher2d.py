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
import copy

from l_point2d import LPoint2D, ptype
from pose2d import Pose2D
from scan2d import Scan2D
from point_cloud_map import PointCloudMap
from covariance_calculator import CovarianceCalculator
from ref_scan_maker import RefScanMaker
from scan_point_resampler import ScanPointResampler
from scan_point_analyser import ScanPointAnalyser
from pose_estimator import PoseEstimatorICP
from pose_fuser import PoseFuser


# ICPuseToScanMatching
class ScanMatcher2D:
    def __init__(
        self,
        cnt=-1,
        prevScan=None,
        initPose=None,
        scthre=1.0,
        nthre=50,
        dgcheck=False,
        pcmap=None,
        spres=None,
        spana=None,
        estim=None,
        rsm=None,
        pfu=None,
        cov=None,
    ):
        self.cnt = cnt  # supportsLogicalTimeScanNumber
        self.prevScan = prevScan if prevScan else Scan2D()  # onePreviousScan
        self.initPose = initPose if initPose else Pose2D()  # thePositionOfTheOriginOfTheMapNormal (0,0,0)
        self.scthre = scthre  # scoreThresholdIfItIsBigger,ItWillBeConsideredAnIcpFailure
        self.nthre = nthre  # usageScoreThresholdIfItIsSmaller,ItWillBeConsideredAnIcpFailure
        self.dgcheck = dgcheck  # doYouWantToPerformDegeneration?
        self.pcmap = pcmap if pcmap else PointCloudMap()  # pointGroupMap
        self.spres = spres if spres else ScanPointResampler()  # scanningIntervalUniform
        self.spana = spana if spana else ScanPointAnalyser()  # scanPointLineCalculation
        self.estim = estim if estim else PoseEstimatorICP()  # robotPositionEstimate
        self.rsm = rsm if rsm else RefScanMaker()  # referenceScanGeneration
        self.pfu = pfu if pfu else PoseFuser()  # Sensor fusioner
        self.cov = cov if cov else np.eye(3)  # Robot migration amount of coordinates

    def setRefScanMaker(self, r):
        self.rsm = r
        if len(self.pcmap) != 0:
            self.rsm.setPointCloudMap(self.pcmap)

    def setPointCloudMap(self, m):
        self.pcmap = m
        self.rsm.setPointCloudMap(self.pcmap)

    def reset(self):
        self.cnt = -1

    def setDgCheck(self, t):
        self.dgcheck = t

    # Execution of scan matching
    def matchScan(self, curScan):
        self.cnt = self.cnt + 1
        self.spres.resamplePoints(curScan)  # Uniform of scanning point intervals
        self.spana.analysePoints(curScan.lps)  # Calculate the normal scanning point
        # The first scan is simply put on the map
        if self.cnt == 0:
            self.growMap(curScan, self.initPose)
            self.prevScan = curScan  # Immediately before scan settings
            return True
        # Calculate the amount of movement using the odometri value in the data file
        odoMotion = Pose2D()  # For moving amount based on odometry
        odoMotion = curScan.pose.calRelativePose(self.prevScan.pose, odoMotion)  # The relative position with the previous scan is moving
        lastPose = self.pcmap.getLastPose()  # Just before
        predPose = Pose2D()  # For predicted position by odometry
        predPose = Pose2D.calGlobalPose(odoMotion, lastPose, predPose)  # Add the moving amount to the right position to get the predicted position
        refScan = self.rsm.makeRefScanLM()  # Use the group of the reference scan production map
        
        self.estim.setScanPair_scan2d_GT(curScan, refScan)  # Set scanning to ICP

        estPose = Pose2D()  # For estimated position by ICP
        score, estPose = self.estim.estimatePose(predPose, estPose)  # Execute the ICP with the predicted position as the initial value
        usedNum = self.estim.getUsedNum()

        # Whether you succeeded in scan matching
        if score <= self.scthre and usedNum >= self.nthre:  # If the score is smaller than the threshold, it will be successful
            successful = True
        else:
            successful = False

        if self.dgcheck:  # When dealing with degeneration
            if successful:
                fusedPose = Pose2D()  # Fusion result
                fusedCov = np.eye(3)  # For coordination after sensor fusion
                self.pfu.setRefScan(refScan)
                # Combine ICP results and odometry values ​​with sensor fusion instrument PFU
                estPose, self.cov = self.pfu.fusePose(curScan, estPose, odoMotion, lastPose, fusedPose, fusedCov)
            else:  # If the ICP is not successful, use the predicted position by the odometri
                estPose = predPose
        else:  # If you do not deal with degeneration (basically use the estimated position by ICP)
            if not successful:
                estPose = predPose  # If you cannot use ICP, use the predicted position by the odometry
        self.growMap(curScan, estPose)  # Added scan points to the map
        self.prevScan = copy.deepcopy(curScan)  # Immediately before scan settings

        return successful

    # Add a scan now to grow the map    
    def growMap(self, scan, pose):
        lps = scan.lps  # Scan points (robot coordinate system)
        R = pose.Rmat  # Estimated robot position
        tx = pose.tx
        ty = pose.ty

        scanG_list = list()
        for i in range(len(lps)):
            lp = lps[i]
            if lp.type == ptype.ISOLATE:  # Excludes isolation points (without nerms)
                continue
            x = R[0, 0] * lp.x + R[0, 1] * lp.y + tx  #Convert to a map coordinate system
            y = R[1, 0] * lp.x + R[1, 1] * lp.y + ty
            nx = R[0, 0] * lp.nx + R[0, 1] * lp.ny  # Converts the normal vector
            ny = R[1, 0] * lp.nx + R[1, 1] * lp.ny

            mlp = LPoint2D(self.cnt, x, y)  # Create a new point
            mlp.setNormal(nx, ny)
            mlp.setType(lp.type)
            scanG_list.append(mlp)
        scanG = np.asarray(scanG_list)

        # Registered in the point group map PCMAP
        self.pcmap.addPose(pose)
        self.pcmap.addPoints(scanG)
        self.pcmap.setLastScan(scan)  # Save for reference scan
        self.pcmap.makeLocalMap()  # Generate local maps
        self.pcmap.setLastPose(pose)
