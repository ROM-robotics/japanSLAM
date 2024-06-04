#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import numpy as np
import time
import sys

from l_point2d import LPoint2D
from pose2d import Pose2D
from scan2d import Scan2D
from sensor_data_reader import SensorDataReader
from point_cloud_map import PointCloudMap
from map_drawer import MapDrawer
from slam_front_end import SlamFrontEnd


class SlamLauncher:
    def __init__(self, startN=0, drawSkip=10, odometryOnly=False, ipose=None, lidarOffset=None, sreader=None, pcmap=None, sfront=None, mdrawer=None): # RasPiMouse drawskip 10
        self.startN = startN  # Start scan number
        self.drawSkip = drawSkip  # Drawing interval
        self.odometryOnly = odometryOnly  # Flag to build maps o...
        self.ipose = ipose if ipose else Pose2D()  #  Auxiliary data for b...
        #  Mounting position of...
        self.lidarOffset = lidarOffset if lidarOffset else Pose2D()
        # Load sensor data from file
        self.sreader = sreader if sreader else SensorDataReader()
        self.pcmap = pcmap if pcmap else PointCloudMap()  # Point cloud
        self.sfront = sfront if sfront else SlamFrontEnd()  # SLAM front end
        self.mdrawer = mdrawer if mdrawer else MapDrawer()  #  Drawing with gnuplot  

    def setStartN(self, n):
        self.startN = n

    def setOdometryOnly(self, p):
        self.odometryOnly = p

    def run(self, inFile):
        self.mdrawer.setAspectRatio(1.0)  # The ratio of the x axis and y axis when drawing
        cnt = 0  # logical time of processing
        if self.startN > 0:
            self.skipData(inFile, self.startN)  # skip data up to startN
        scan = Scan2D()
        #  load on scan from file
        eof = self.sreader.loadScan(inFile, cnt, scan)
        while eof is False:
            if self.odometryOnly:  # map construction using odometry
                if cnt == 0:
                    self.ipose = scan.pose
                    self.ipose.calRmat()
                self.mapByOdometry(scan)
            else:
                self.sfront.process(scan)  # SLAM  Map construction ( လိုင်း ၅၈ )
                self.pcmap = self.sfront.pcmap
            if cnt % self.drawSkip == 0:  # drawSkip Draw the result every others
                self.mdrawer.drawMapGp(self.pcmap)
            cnt = cnt + 1  # Logical time update
            eof = self.sreader.loadScan(inFile, cnt, scan)  #  load next scan 
            print("---- SlamLauncher: cnt=%d ends ----\n" % cnt)
        self.sreader.closeScanFile(inFile)
        print("pose %f %f %f" %(self.pcmap.poses[-1].tx,self.pcmap.poses[-1].ty,self.pcmap.poses[-1].th))
        print("SlamLauncher finished.")

        if sys.platform != 'darwin':
            input()  # Wait for some kind of time

    # skip up to num scans from the start
    def skipData(self, inFile, num):
        scan = Scan2D()
        self.sreader.loadScan(inFile, 0, scan, skip=True)
        for i in range(num):  # num blank reading
            self.sreader.loadScan(inFile, 0, scan, skip=True)

    # map construction using odometry
    def mapByOdometry(self, scan):
        pose = scan.pose
        lps = scan.lps  # Scan point cloud
        glps_list = list()
        for i in range(len(lps)):
            lp = lps[i]
            glp = LPoint2D()
            pose.globalPoint_io(lp, glp)  # Converted from sensor coordinate system to map coordinate system
            glps_list.append(glp)
        glps = np.asarray(glps_list)

        # storing data to point cloud map pcmap
        self.pcmap.addPose(pose)
        self.pcmap.addPoints(glps)
        self.pcmap.makeGlobalMap()

    # Scan drawing
    def showScans(self, inFile):
        self.mdrawer.setRange(6)  # Drawing range. If the scan is 6m square
        self.mdrawer.setAspectRatio(1.0)  # The ratio of the x axis and y axis when drawing
        cnt = 0  # logical time of processing 
        if self.startN > 0:
            self.skipData(inFile, self.startN)  # skip data up to startN
        scan = Scan2D()
        eof = self.sreader.loadScan(inFile, cnt, scan)
        while eof is False:
            time.sleep(0.1)  # drawing is too fast, increase the drawing interval
            self.mdrawer.drawScanGp(scan)  # Draw scan data
            print("---- scan num=%d ----" % cnt)
            eof = self.sreader.loadScan(inFile, cnt, scan)
            cnt = cnt + 1
        self.sreader.closeScanFile(inFile)
        print("SlamLauncher finished.")

    # Scan reading
    def setFilename(self, filename):
        flag = self.sreader.openScanFile(filename)  # Open file
        return flag


def main():
    argvs = sys.argv
    argc = len(argvs)
    if argc != 3:
        print("HowToWrite for this program: python slam_lancher.py FILE_NAME startN")
        return
    sl = SlamLauncher()
    inFile = sl.setFilename(sys.argv[1])
    startN = int(sys.argv[2])
    sl.setStartN(startN)
    print("data file: %s" % sys.argv[1])
    print("startN: %d" % startN)
    #    sl.showScans(inFile)
    #sl.setOdometryOnly(True)
    sl.setOdometryOnly(False)
    sl.run(inFile)

    
if __name__ == "__main__":
    main()
