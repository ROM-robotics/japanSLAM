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

from pose2d import Pose2D
from scan2d import Scan2D
from nn_grid_table import NNGridTable


# Partial map
class Submap:
    def __init__(self, atdS=0.0, cntS=0, cntE=-1, mps=None):
        self.atdS = atdS  # Cumulative mileage at the starting point of partial map
        self.cntS = cntS  # The first scan number on the part of the map
        self.cntE = cntE  # Last scan number of partial map
        self.mps = mps if mps else np.empty(0)  # Scan points in partial maps vector<LPoint2D>

    def addPoints(self, lps):
        mps_list = self.mps.tolist()
        for i in range(len(lps)):
            mps_list.append(lps[i])
        self.mps = np.asarray(mps_list)

    # Get a representative point of partial maps using a lattice table
    def subsamplePoints(self, nthre):
        nntab = NNGridTable()  # Lattice table
        for i in range(len(self.mps)):
            lp = self.mps[i]
            nntab.addPoint(lp)  # Register all points
        sps = np.empty(0)
        sps = nntab.makeCellPoints(nthre, sps)  # NTHRE puts the representative point of cells or more in SPS
        return sps

# Base class of the point group map
class PointCloudMap:
    MAX_POINT_NUM = 1000000  # Maximum score of globalmap

    def __init__(
        self,
        nthre=5, # nthre=0,
        poses=None,
        lastPose=None,
        lastScan=None,
        globalMap=None,
        localMap=None,
        nntab=None,
        atdThre=5.,
        atd=0.,
        submaps=None
    ):
        # Lattice table cell score threshold(GT and LP only)        
        self.nthre = nthre
        # Robot trajectory
        self.poses = poses if poses else np.empty([0, 0])
        # Lastly estimated robot position
        self.lastPose = lastPose if lastPose else Pose2D()
        # Finally processed scan
        self.lastScan = lastScan if lastScan else Scan2D()
        # Whole map. Points after thinning
        self.globalMap = globalMap if globalMap else np.empty(self.MAX_POINT_NUM)
        # Local map near the current location. Used for scan matching
        self.localMap = localMap if localMap else np.empty(0)
        # for Point group map using a lattice table
        self.nntab = nntab if nntab else NNGridTable()
        # Cumulative mileage that is a part of the part map(atd)[m]        
        self.atdThre = atdThre
        # Current cumulative mileage(accumulated travel distance)        
        self.atd = atd
        # Partial map
        self.submaps = submaps if submaps else np.array([Submap()])

    def setLastPose(self, pose2d):
        self.lastPose = copy.deepcopy(pose2d)

    def getLastPose(self):
        return self.lastPose

    def setLastScan(self, scan2d):
        self.lastScan = copy.deepcopy(scan2d)

    # Added robot position
    def addPose(self, pose2d):
        # Cumulative mileage(atd)Calculation of
        if len(self.poses) > 0:
            pp = self.poses[-1]
            self.atd = self.atd + math.sqrt((pose2d.tx - pp.tx) * (pose2d.tx - pp.tx) + (pose2d.ty - pp.ty) * (pose2d.ty - pp.ty))
        else:
            self.atd = self.atd + math.sqrt(pose2d.tx * pose2d.tx + pose2d.ty * pose2d.ty)
        self.poses = np.append(self.poses, copy.deepcopy(pose2d))

    # Addition of scan points LP
    def addPoints(self, vector_l_point2d):
        curSubmap = self.submaps[-1]  # Current partial map
        if self.atd - curSubmap.atdS >= self.atdThre:  # If the cumulative mileage exceeds the threshold, change to a new part map
            size = len(self.poses)
            curSubmap.cntE = size - 1  # Last scan number of partial map
            curSubmap.mps = curSubmap.subsamplePoints(self.nthre)  # The finished part map is only a representative point (lighter weight)
            submap = Submap(self.atd, size)  # New part map
            submap.addPoints(vector_l_point2d)  # Registration of scan points
            self.submaps = np.append(self.submaps, submap)  # Add partial map
        else:
            curSubmap.addPoints(vector_l_point2d)  # Added point group to the current part map

    # Generation of overall maps. It is faster to make local maps here LP
    def makeGlobalMap(self):
        self.globalMap = np.empty(self.MAX_POINT_NUM)  # Initialization
        self.localMap = np.empty(0)  # Initialization
        globalMap_list = list()
        localMap_list = list()

        # Collect points from the already fixed part maps other than the present
        num = len(self.submaps) - 1
        for i in range(num):
            submap = self.submaps[i]  # Partial map
            mps = submap.mps  # Part of the part of the map. It is only a representative point
            num2 = len(mps)
            for j in range(num2):
                globalMap_list.append(mps[j])  # Put all points on the whole map
            if i == len(self.submaps) - 2:  # Put only the last part map in the local map
                for j in range(num2):
                    localMap_list.append(mps[j])

        # Put the representative point of the current part map in the whole map and local map
        curSubmap = self.submaps[-1]  # Current partial map
        sps = curSubmap.subsamplePoints(self.nthre)  # Get a representative point
        for i in range(len(sps)):
            globalMap_list.append(sps[i])  # Put all points on the whole map
            localMap_list.append(sps[i])
        self.globalMap = np.asarray(globalMap_list)
        self.localMap = np.asarray(localMap_list)

    # Local map generation LP
    def makeLocalMap(self):
        self.localMap = np.empty(0)  # Initialization
        localMap_list = list()
        if len(self.submaps) >= 2:
            submap = self.submaps[len(self.submaps) - 2]  # Use only the previous part map
            mps = submap.mps  # Part of the part of the map. It is only a representative point
            num = len(mps)
            localMap_list = [mps[i] for i in range(num)]
        # Put the representative point of the current part map in the local map
        curSubmap = self.submaps[-1]  # Current partial map
        sps = curSubmap.subsamplePoints(self.nthre)  # Get a representative point
        for i in range(len(sps)):
            localMap_list.append(sps[i])
        self.localMap = np.asarray(localMap_list)
