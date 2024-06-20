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


# 格子テーブル
class NNGridTable:
    def __init__(self, csize=0.05, rsize=40.):
        self.csize = csize  # セルサイズ[m]
        self.rsize = rsize  # 対象領域のサイズ[m]. 正方形の1辺の半分.
        self.tsize = int(self.rsize / self.csize)  # テーブルサイズの半分
        self.table = {}

        self.dthre = 0.2  # これより遠い点は除外する[m]
        R = int(self.dthre / self.csize)  # 先に計算しておく
        self.r_range = range(-R, R)
        self.set_r_coords()  # 予め全座標セット
        self.dthre_dthre = self.dthre * self.dthre
        self.tsizex2 = 2 * self.tsize

        # for makeCellPoints
        self.gx = 0.  # 点群の重心位置
        self.gy = 0.
        self.nx = 0.  # 点群の法線ベクトルの平均
        self.ny = 0.
        self.sid = 0
        self.line = ptype.LINE

    def set_r_coords(self):
        # range(-R, R)の2次元全座標
        xx, yy = np.meshgrid(self.r_range, self.r_range)
        self.r_coords = np.c_[xx.flatten(), yy.flatten()].tolist()

    def clear(self):
        # 各セルを空にする 内包表記Pythonリストのが速い
        [c.clear() for c in self.table.values()]

    # 格子テーブルにスキャン点lpを登録する
    def addPoint(self, lp):
        # テーブル検索のインデックス計算. まず対象領域内にあるかチェックする.
        xi = int(lp.x / self.csize) + self.tsize
        if xi < 0 or xi > self.tsizex2:  # 対象領域の外
            return
        yi = int(lp.y / self.csize) + self.tsize
        if yi < 0 or yi > self.tsizex2:  # 対象領域の外
            return
        idx = int(yi * (self.tsizex2 + 1) + xi)  # テーブルのインデックス
        if idx not in self.table:
            self.table[idx] = NNGridCell()
        self.table[idx].lps.append(lp)  # 目的のセルに入れる

    # スキャン点clpをpredPoseで座標変換した位置に最も近い点を格子テーブルから見つける
    def findClosestPoint(self, clp, predPose):
        glp = LPoint2D()  # clpの予測位置
        predPose.globalPoint_io(clp, glp)  # relPoseで座標変換
        # clpのテーブルインデックス. 対象領域内にあるかチェックする.
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
        if my_dmin >= my_dmin > self.dthre_dthre:
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
        self.sid /= num  # スキャン番号の平均とる場合
        newLp = LPoint2D(self.sid, self.gx, self.gy)  # セルの代表点を生成
        newLp.setNormal(self.nx, self.ny)  # 法線ベクトル設定
        newLp.setType(self.line)  # タイプは直線にする
        ps_list.append(newLp)

    # 格子テーブルの各セルの代表点を作ってpsに格納する。
    # 現状はセル内の各点のスキャン番号の平均をとる。
    def makeCellPoints(self, nthre, ps):
        ps_list = ps.tolist()
        lps_list = [
            c.lps  # セルのスキャン点群
            # 点数がnthreより多いセルだけ処理する
            for c in self.table.values() if len(c.lps) >= nthre
        ]
        for lps in lps_list:
            self.do_in_lps(lps, ps_list)
        return np.asarray(ps_list)
