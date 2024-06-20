#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Shuro Nakajima from the following C++ software:
#  LittleSLAM (https://github.com/furo-org/LittleSLAM) written by Masahiro Tomono,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import numpy as np

from pose2d import Pose2D


# ポーズグラフの頂点
class PoseNode:
    def __init__(self, nid=-1, pose=None, arcs=None):
        self.nid = nid
        self.pose = pose if pose else Pose2D()
        self.arcs = arcs if arcs else np.empty(0)

    def setPose(self, pose):
        self.pose = pose

    def setNid(self, n):
        self.nid = n

    def addArc(self, arc):
        self.arcs = np.append(self.arcs, arc)


# ポーズグラフの辺
class PoseArc:
    def __init__(self, src=None, dst=None, relPose=None, inf=None):
        self.src = src  # このアークの始点側のノード
        self.dst = dst  # このアークの終点側のノード
        self.relPose = relPose if relPose else Pose2D()  # このアークのもつ相対位置(計測値)
        self.inf = inf if inf else np.eye(3)  # 情報行列

    def setup(self, s, d, rel, inf):
        self.src = s
        self.dst = d
        self.relPose = rel
        self.inf = inf


# ポーズグラフ
class PoseGraph:
    POOL_SIZE = 100000

    def __init__(self, nodePool=None, arcPool=None, nodes=None, arcs=None):
        self.nodePool = nodePool if nodePool else np.empty(0)
        self.arcPool = arcPool if arcPool else np.empty(0)
        self.nodes = nodes if nodes else np.empty(0)
        self.arcs = arcs if arcs else np.empty(0)

    def reset(self):
        for i in range(len(self.nodes)):
            self.nodes = np.delete(self.nodes, 0, 0)
        for i in range(len(self.arcs)):
            self.arcs = np.delete(self.arcs, 0, 0)
        for i in range(len(self.nodePool)):
            self.nodePool = np.delete(self.nodePool, 0, 0)
        for i in range(len(self.arcPool)):
            self.arcPool = np.delete(self.arcPool, 0, 0)

    # ノードの生成
    def allocNode(self):
        if len(self.nodePool) >= PoseGraph.POOL_SIZE:
            print("Error: exceeds nodePool capacity %d" % len(self.nodePool))
            return None
        node = PoseNode()
        self.nodePool = np.append(self.nodePool, node)  # メモリプールに追加して, それを参照する。
        return self.nodePool[-1]

    # アークの生成
    def allocArc(self):
        if len(self.arcPool) >= PoseGraph.POOL_SIZE:
            print("Error: exceeds arcPool capacity")
            return None
        arc = PoseArc()
        self.arcPool = np.append(self.arcPool, arc)  # メモリプールに追加して, それを参照する。
        return self.arcPool[-1]

    # グラフ生成
    # ポーズグラフにノード追加
    def addNode(self, pose):
        n1 = self.allocNode()  # ノード生成
        self.addNode2(n1, pose)  # ポーズグラフにノード追加
        return n1

    # ポーズグラフにノード追加
    def addNode2(self, n1, pose):
        n1.setNid(len(self.nodes))  # ノードID付与. ノードの通し番号と同じ
        n1.setPose(pose)  # ロボット位置を設定
        self.nodes = np.append(self.nodes, n1)  # nodesの最後に追加

    # ノードID(nid)からノード実体を見つける
    def findNode(self, nid):
        for i in range(len(self.nodes)):  # 単純に線形探索
            n = self.nodes[i]
            if n.nid == nid:  # nidが一致したら見つけた
                return n
        return None

    # ポーズグラフにアークを追加する
    def addArc(self, arc):
        arc.src.addArc(arc)  # 終点ノードにarcを追加
        arc.dst.addArc(arc)  # 終点ノードにarcを追加
        self.arcs = np.append(self.arcs, arc)  # arcsの最後にarcを追加

    # 始点ノードsrcNidと終点ノードdstNidの間にアークを生成する
    def makeArc(self, srcNid, dstNid, relPose, cov):
        inf = np.linalg.inv(cov)  # infはcovの逆行列
        src = self.nodes[srcNid]  # 始点ノード
        dst = self.nodes[dstNid]  # 終点ノード
        arc = self.allocArc()  # アークの生成
        arc.setup(src, dst, relPose, inf)  # relPoseは計測による相対位置
        return arc

    # 始点ノードがsrcNid、終点ノードがdstNidであるアークを返す
    def findArc(self, srcNid, dstNid):
        for i in range(len(self.arcs)):
            a = self.arcs[i]
            if a.src.nid == srcNid and a.dst.nid == dstNid:
                return a
        return None
