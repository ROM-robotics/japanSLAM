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
        self.startN = startN  # 開始スキャン番号
        self.drawSkip = drawSkip  # 描画間隔
        self.odometryOnly = odometryOnly  # 地図構築をオドメトリだけで行うかのフラグ
        self.ipose = ipose if ipose else Pose2D()  # オドメトリ地図構築の補助データ
        # ロボット中心からのセンサの取付位置
        self.lidarOffset = lidarOffset if lidarOffset else Pose2D()
        # ファイルからのセンサデータ読み込み
        self.sreader = sreader if sreader else SensorDataReader()
        self.pcmap = pcmap if pcmap else PointCloudMap()  # 点群地図
        self.sfront = sfront if sfront else SlamFrontEnd()  # SLAMフロントエンド
        self.mdrawer = mdrawer if mdrawer else MapDrawer()  # gnuplotによる描画

    def setStartN(self, n):
        self.startN = n

    def setOdometryOnly(self, p):
        self.odometryOnly = p

    def run(self, inFile):
        self.mdrawer.setAspectRatio(1.0)  # The ratio of the X -axis and the Y -axis when drawing
        cnt = 0  # Processing logical time
        if self.startN > 0:
            self.skipData(inFile, self.startN)  # Read the data to STARTN
        scan = Scan2D()
        #  Read one scan from the file
        eof = self.sreader.loadScan(inFile, cnt, scan)
        while eof is False:
            if self.odometryOnly:  # Memorial construction by odmetry (priority over SLAM)
                if cnt == 0:
                    self.ipose = scan.pose
                    self.ipose.calRmat()
                self.mapByOdometry(scan)
            else:
                self.sfront.process(scan)  # Machine construction by SLAM
                self.pcmap = self.sfront.pcmap
            if cnt % self.drawSkip == 0:  # Drawskip draws results everywhere
                self.mdrawer.drawMapGp(self.pcmap)
            cnt = cnt + 1  # Logical time update
            eof = self.sreader.loadScan(inFile, cnt, scan)  # Read the following scan
            print("---- SlamLauncher: cnt=%d ends ----\n" % cnt)
        self.sreader.closeScanFile(inFile)
        print("pose %f %f %f" %(self.pcmap.poses[-1].tx,self.pcmap.poses[-1].ty,self.pcmap.poses[-1].th))
        print("SlamLauncher finished.")

        if sys.platform != 'darwin':
            input()  # Wait for some kind of input to leave the drawing screen even after the processing

    # Read from the start to the N -individual scan
    def skipData(self, inFile, num):
        scan = Scan2D()
        self.sreader.loadScan(inFile, 0, scan, skip=True)
        for i in range(num):  # num個空読みする
            self.sreader.loadScan(inFile, 0, scan, skip=True)

    # オドメトリのよる地図構築
    def mapByOdometry(self, scan):
        pose = scan.pose
        lps = scan.lps  # スキャン点群
        glps_list = list()
        for i in range(len(lps)):
            lp = lps[i]
            glp = LPoint2D()
            pose.globalPoint_io(lp, glp)  # センサ座標系から地図座標系に変換
            glps_list.append(glp)
        glps = np.asarray(glps_list)

        # 点群地図pcmapにデータを格納
        self.pcmap.addPose(pose)
        self.pcmap.addPoints(glps)
        self.pcmap.makeGlobalMap()

    # スキャン描画
    def showScans(self, inFile):
        self.mdrawer.setRange(6)  # 描画範囲。スキャンが6m四方の場合
        self.mdrawer.setAspectRatio(1.0)  # 描画時のx軸とy軸の比
        cnt = 0  # 処理の論理時刻
        if self.startN > 0:
            self.skipData(inFile, self.startN)  # startNまでデータを読み飛ばす
        scan = Scan2D()
        eof = self.sreader.loadScan(inFile, cnt, scan)
        while eof is False:
            time.sleep(0.1)  # 描画が速すぎるため，描画間隔をあける
            self.mdrawer.drawScanGp(scan)  # スキャンデータの描画
            print("---- scan num=%d ----" % cnt)
            eof = self.sreader.loadScan(inFile, cnt, scan)
            cnt = cnt + 1
        self.sreader.closeScanFile(inFile)
        print("SlamLauncher finished.")

    # スキャン読み込み
    def setFilename(self, filename):
        flag = self.sreader.openScanFile(filename)  # ファイルをオープン
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
    #   sl.showScans(inFile)
    #sl.setOdometryOnly(True)
    sl.setOdometryOnly(False)
    sl.run(inFile)

    
if __name__ == "__main__":
    main()
