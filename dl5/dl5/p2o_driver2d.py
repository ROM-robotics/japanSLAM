#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import numpy as np

from my_util import DEG2RAD, RAD2DEG
from pose2d import Pose2D
from pose_graph import PoseGraph
from p2o2d import Pose2D_p2o, Con2D, Optimizer2D

# ポーズグラフ最適化ライブラリkslamを起動する。
class P2oDriver2D:
    def __init__(self):
        pass

    def doP2o(self, pg=None, newPoses=None, N=0):
        pg = pg if pg else PoseGraph()
        newPoses = newPoses if newPoses else np.empty(0)
        nodes = pg.nodes  # ポーズノード
        arcs = pg.arcs  # ポーズアーク

        # ポーズノードをp2o用に変換
        pnodes = np.empty(0)  # p2oのポーズノード集合
        len_nodes = len(nodes)
        np_append = np.append
        for num in range(0, len_nodes, 1):
            node = nodes[num]
            pose = node.pose  # ノードの位置
            pnodes = np_append(pnodes, Pose2D_p2o(pose.tx, pose.ty, DEG2RAD(pose.th)))  # 位置だけ入れる
        # ポーズアークをkslam用に変換
        pcons = np.empty(0)  # p2oのポーズアーク集合
        for num in range(len(arcs)):
            arc = arcs[num]
            src = arc.src
            dst = arc.dst
            relPose = arc.relPose
            con = Con2D()
            if src is not None:
                con.id1 = src.nid
            else:
                con.id1 = 0
            if dst is not None:
                con.id2 = dst.nid
            else:
                con.id2 = 0
            con.t = Pose2D_p2o(relPose.tx, relPose.ty, DEG2RAD(relPose.th))
            for i in range(0, 3, 1):
                for j in range(0, 3, 1):
                    con.info_mat[i, j] = arc.inf[i, j]
            pcons = np_append(pcons, con)  # 位置だけ入れる

        opt = Optimizer2D()  # p2oインスタンス
        result = opt.optimize_path(pnodes, pcons, N, 3)  # N回実行

        # 結果をnewPoseに格納する
        for num in range(len(result)):
            newPose = result[num]  # i番目のノードの修正された位置
            pose = Pose2D(newPose.x, newPose.y, RAD2DEG(newPose.theta))
            newPoses = np_append(newPoses, pose)
        return newPoses
