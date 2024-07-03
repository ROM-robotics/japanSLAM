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
import itertools

from l_point2d import LPoint2D, ptype


class NNGridCell:
    def __init__(self, lps=None):
        if lps is None:
            self.lps = []
        else:
            self.lps = list(lps)

    def clear(self):  # 空にする
        self.lps = []


# Lattice table
class NNGridTable:
    def __init__(self, csize=0.05, rsize=40.):
        self.csize = csize  # Cell size[m]
        self.rsize = rsize  # Target area size[m]. Half of one side of the square.
        self.tsize = int(self.rsize / self.csize)  # Half of the table size
        self.table = {}

        self.dthre = 0.2  # Exclude points farther than this[m]
        R = int(self.dthre / self.csize)  # Calculate first
        self.r_range = range(-R, R)
        self.set_r_coords()  # All coordinates set in advance
        self.dthre_dthre = self.dthre * self.dthre
        self.tsizex2 = 2 * self.tsize

        # for makeCellPoints
        self.gx = 0.  # Parked center of gravity of the point group
        self.gy = 0.
        self.nx = 0.  # Average of the normal vector of the point group
        self.ny = 0.
        self.sid = 0
        self.line = ptype.LINE

    def set_r_coords(self):
        # range(-R, R)の2次元全座標
        xx, yy = np.meshgrid(self.r_range, self.r_range)
        self.r_coords = np.c_[xx.flatten(), yy.flatten()].tolist()

    def clear(self):        
        [c.clear() for c in self.table.values()]  # 各セルを空にする

    # Register a scanning point LP in the lattice table
    def addPoint(self, lp):
        # Table search index calculation. First, check if it is in the target area
        xi = int(lp.x / self.csize) + self.tsize
        if xi < 0 or xi > self.tsizex2:  # Outside of the target area
            return
        yi = int(lp.y / self.csize) + self.tsize
        if yi < 0 or yi > self.tsizex2:  # Outside of the target area
            return
        idx = int(yi * (self.tsizex2 + 1) + xi)  # Table index
        if idx not in self.table:
            self.table[idx] = NNGridCell()            
        self.table[idx].lps.append(lp)  # Put it in the desired cell

    # Find the point closest to the position where the scanning point CLP is coordinated by predpose from the grid table
    def findClosestPoint(self, clp, predPose):
        glp = LPoint2D()  # clpの予測位置
        predPose.globalPoint_io(clp, glp)  # relPoseで座標変換
        # clpのテーブルインデックス. 対象領域内にあるかチェック
        cxi = int(glp.x / self.csize) + self.tsize
        if cxi < 0 or cxi > self.tsizex2:
            return None
        cyi = int(glp.y / self.csize) + self.tsize
        if cyi < 0 or cyi > self.tsizex2:
            return None

        x_min = 0 - cxi
        x_max = self.tsizex2 - cxi
        y_min = 0 - cyi
        y_max = self.tsizex2 - cyi
        # 範囲内の座標のみでテーブルインデックス計算
        idxs = [
            (cyi + y) * (self.tsizex2 + 1) + (cxi + x)
            for x, y in self.r_coords
            if x_min <= x <= x_max
            and y_min <= y <= y_max
        ]
        # セルがもつスキャン点群のリスト
        lps_list = [self.table[idx].lps for idx in idxs if idx in self.table]
        if not lps_list:
            return None
        # 1次元化
        lp_list = list(itertools.chain.from_iterable(lps_list))
        if not lp_list:
            return None
        # 距離のリスト
        dists = [
            (lp.x - glp.x) ** 2 + (lp.y - glp.y) ** 2
            for lp in lp_list
        ]
        my_dmin = min(dists)
        if my_dmin > self.dthre_dthre:
            return None
        my_dmin_i = dists.index(my_dmin)
        return lp_list[my_dmin_i]

    def do_in_lp(self, lp):
        self.gx += lp.x  # 位置を累積
        self.gy += lp.y
        self.nx += lp.nx  # 法線ベクトル成分を累積
        self.ny += lp.ny
        self.sid += lp.sid  # スキャン番号の平均とる場合

    def do_in_lps(self, lps, ps_list):
        num = len(lps)
        self.gx = 0.  # 点群の重心位置
        self.gy = 0.
        self.nx = 0.  # 点群の法線ベクトルの平均
        self.ny = 0.
        self.sid = 0
        
        for j in range(num):
            self.do_in_lp(lps[j])
        self.gx /= num  # 平均
        self.gy /= num
        L = math.sqrt(self.nx * self.nx + self.ny * self.ny)
        if L == 0 :
            L = 0.000001
        self.nx = self.nx / L  # 平均（正規化）
        self.ny = self.ny / L
        self.sid /= num  # スキャン番号の平均とる場合（このsidは実際には使っていないので，暫定的に平均を取る程度）
        newLp = LPoint2D(self.sid, self.gx, self.gy)  # セルの代表点を生成
        newLp.setNormal(self.nx, self.ny)  # 法線ベクトル設定
        newLp.setType(self.line)  # タイプは直線にする
        ps_list.append(newLp)

    # Create a representative point of each cell of the lattice table and store it in PS
    # At present, the average of the scan number in each point in the cell is taken
    def makeCellPoints(self, nthre, ps):
        ps_list = ps.tolist()
        lps_list = [
            c.lps  # Cell scanning group
            # Process only cells that have more scores than NTHRE
            for c in self.table.values() if len(c.lps) >= nthre
        ]
        for lps in lps_list:
            self.do_in_lps(lps, ps_list)
        return np.asarray(ps_list)
