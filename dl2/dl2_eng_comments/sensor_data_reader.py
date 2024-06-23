#!/usr/bin/python
# coding: utf-8

import numpy as np

from my_util import RAD2DEG
from l_point2d import LPoint2D

class SensorDataReader:
    def __init__(self, angleOffset=0., filepath=None):
        self.angleOffset = angleOffset
        self.filepath = filepath if filepath else ''

    def openScanFile(self, filepath):
        try:
            inFile = open(filepath)
        except OSError:
            print('cannot open', filepath)
        return inFile

    def closeScanFile(self, inFile):
        inFile.close()

    def setAngleOffset(self, angleOffset):
        self.angleOffset = angleOffset

    # Read one line from the data file and set it to each variable.
    # Return False in the last line of the file.    
    def loadScan(self, inFile, cnt, scan2d, skip=False):
        isScan = inFile.readline()
        if not isScan:
            return True  # file end
        if skip:
            return False
        data = isScan.split()
        if data[0] == "LASERSCAN":
            scan2d.setSid(cnt)
            pnum = int(data[4])

            lps = list()
            angle = data[5:(pnum) * 2 + 5:2]
            angle = np.array(angle, dtype=float) + self.angleOffset
            range_data = np.array(data[6:(pnum) * 2 + 6:2], dtype=float)
            for i, d_angle in enumerate(angle):
                if range_data[i] <= scan2d.MIN_SCAN_RANGE or range_data[i] >= scan2d.MAX_SCAN_RANGE:
                    continue
                lp = LPoint2D()
                lp.setSid(cnt)
                lp.calXY(range_data[i], angle[i])
                lps.append(lp)
            scan2d.setLps(lps)
            scan2d.pose.tx = float(data[(pnum) * 2 + 5])
            scan2d.pose.ty = float(data[(pnum) * 2 + 6])
            scan2d.pose.setAngle(RAD2DEG(float(data[(pnum) * 2 + 7])))
            scan2d.pose.calRmat()

        return False  # file continue
