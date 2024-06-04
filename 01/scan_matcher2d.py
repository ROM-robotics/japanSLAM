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


# Scan matching using ICP ICP
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
        self.cnt = cnt  # Logical timecaCompatible with scan numberompatible with scan number
        self.prevScan = prevScan if prevScan else Scan2D()  # One previous scanious scan
        self.initPose = initPose if initPose else Pose2D()  # Position of the origin of the map ousuallythe origin of the map. usually(0,0,0)
        self.scthre = scthre  # Score threshold tIf it is bigger than this, it will be considered an ICP failures bigger than this, it will be considered an ICP failure
        self.nthre = nthre  # Usage number threshold valuenuIf it is smaller than this, it will be considered an ICP failurelue. If it is smaller than this, it will be considered an ICP failure
        self.dgcheck = dgcheck  # Do you want to perform degeneration?ant to perform degeneration?
        self.pcmap = pcmap if pcmap else PointCloudMap()  # Point group mapt group map
        self.spres = spres if spres else ScanPointResampler()  # Scanning interval uniformnterval uniform
        self.spana = spana if spana else ScanPointAnalyser()  # Scan -point line calculationnt line calculation
        self.estim = estim if estim else PoseEstimatorICP()  # Robot position estimateition estimate
        self.rsm = rsm if rsm else RefScanMaker()  # Reference scan generatione scan generation
        self.pfu = pfu if pfu else PoseFuser()  # Sensor fusioner fusioner
        self.cov = cov if cov else np.eye(3)  # Robot migration amount of coordinateson amount of coordinates

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

    # Execution of scan matching scan matching
    def matchScan(self, curScan):   # ( လိုင်းနံပါတ် 74 )
        self.cnt = self.cnt + 1
        self.spres.resamplePoints(curScan)  # Uniform of scanning point intervalsanning point intervals
        self.spana.analysePoints(curScan.lps)  # Calculate the normal scanning point normal scanning point
        # The first scan is simply put on the mapsimply put on the map
        if self.cnt == 0:
            self.growMap(curScan, self.initPose)
            self.prevScan = curScan  # Immediately before scan settingsly before scan settings
            return True
        # Calculate the amount of movement using the odometri value in the data filet using the odometri value in the data file
        odoMotion = Pose2D()  # For moving amount based on odometryount based on odometry
        odoMotion = curScan.pose.calRelativePose(self.prevScan.pose, odoMotion)  # The relative position with the previous scan is movingsition with the previous scan is moving ( လိုင်းနံပါတ် 85 )
        lastPose = self.pcmap.getLastPose()  # Just before before
        predPose = Pose2D()  # For predicted position by odometry position by odometry
        predPose = Pose2D.calGlobalPose(odoMotion, lastPose, predPose)  # Add the moving amount to the right position to get the predicted positionnt to the right position to get the predicted position ( လိုင်းနံပါတ် 88 )
        refScan = self.rsm.makeRefScanLM()  # Production of reference scannUse the map of the mapence scan Use the map of the map
        
        self.estim.setScanPair_scan2d_GT(curScan, refScan)  # Set scanning to ICPg to ICP

        estPose = Pose2D()  # For estimated position by ICPed position by ICP
        score, estPose = self.estim.estimatePose(predPose, estPose)  # Execute the ICP with the predicted position as the initial valueith the predicted position as the initial value ( လိုင်းနံပါတ် 94 )
        usedNum = self.estim.getUsedNum()

        # Whether you succeeded in scan matchingded in scan matching
        if score <= self.scthre and usedNum >= self.nthre:  # If the score is smaller than the threshold, it will be successfulaller than the threshold, it will be successful
            successful = True
        else:
            successful = False

        if self.dgcheck:  # When dealing with degenerationng with degeneration ( လိုင်းနံပါတ် 103 )
            if successful:
                fusedPose = Pose2D()  # Fusion resultn result
                fusedCov = np.eye(3)  # For coordination after sensor fusionation after sensor fusion
                self.pfu.setRefScan(refScan)
                # Combine ICP results and odometry values ​​with sensor fusion instrument PFUetry values ​​with sensor fusion instrument PFU
                estPose, self.cov = self.pfu.fusePose(curScan, estPose, odoMotion, lastPose, fusedPose, fusedCov)   # ( လိုင်းနံပါတ်  109 )
            else:  # If the ICP is not successful, use the predicted position by the odometriful, use the predicted position by the odometri
                estPose = predPose
        else:  # If you do not deal with degeneration (basically use the estimated position by ICP)ation (basically use the estimated position by ICP)
            if not successful:
                estPose = predPose  # If you cannot use ICP, use the predicted position by the odometrye the predicted position by the odometry
        self.growMap(curScan, estPose)  # Added scan points to the mapoints to the map ( လိုင်းနံပါတ် 115 )
        self.prevScan = copy.deepcopy(curScan)  # Immediately before scan settingsly before scan settings

        return successful   # # ( လိုင်းနံပါတ် 118 )

    # Add a scan now to grow the mapow the map
    def growMap(self, scan, pose):
        lps = scan.lps  # Scan pointsoRobot coordinate systembot coordinate system)
        R = pose.Rmat  # Estimated robot positionrobot position
        tx = pose.tx
        ty = pose.ty

        scanG_list = list()
        for i in range(len(lps)):
            lp = lps[i]
            if lp.type == ptype.ISOLATE:  # Excludes isolation points (without nerms)lation points (without nerms)
                continue
            x = R[0, 0] * lp.x + R[0, 1] * lp.y + tx  # Convert to a map coordinate systemto a map coordinate system
            y = R[1, 0] * lp.x + R[1, 1] * lp.y + ty
            nx = R[0, 0] * lp.nx + R[0, 1] * lp.ny  # Converts the normal vectorthe normal vector
            ny = R[1, 0] * lp.nx + R[1, 1] * lp.ny

            mlp = LPoint2D(self.cnt, x, y)  # Create a new pointa new point
            mlp.setNormal(nx, ny)
            mlp.setType(lp.type)
            scanG_list.append(mlp)
        scanG = np.asarray(scanG_list)

        # Registered in the point group map PCMAPn the point group map PCMAP
        self.pcmap.addPose(pose)
        self.pcmap.addPoints(scanG)
        self.pcmap.setLastScan(scan)  # Save for reference scaneference scan
        self.pcmap.makeLocalMap()  # Generate local mapse local maps
        self.pcmap.setLastPose(pose)
