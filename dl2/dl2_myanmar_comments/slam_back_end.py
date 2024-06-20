#!/usr/bin/python
# coding: utf-8

import numpy as np

from point_cloud_map import PointCloudMap
from pose_graph import PoseGraph
from p2o_driver2d import P2oDriver2D


class SlamBackEnd:
    def __init__(self, newPoses=None, pcmap=None, pg=None):
        self.newPoses = newPoses if newPoses else np.empty(0)  # Posture after adjusting the pose
        self.pcmap = pcmap if pcmap else PointCloudMap()  # Point group map
        self.pg = pg if pg else PoseGraph()  # Pose graph

    def setPointCloudMap(self, pcmap):
        self.pcmap = pcmap

    def setPoseGraph(self, pg):
        self.pg = pg

    def adjustPoses(self):
        self.newPoses = np.empty(0)  # Initialization
        p2o = P2oDriver2D()
        self.newPoses = p2o.doP2o(self.pg, self.newPoses, 5)  # Five times
        return self.newPoses[-1]

    def remakeMaps(self):  # Posegraph modification
        pnodes = self.pg.nodes
        num = len(self.newPoses)
        for i in range(num):
            npose = self.newPoses[i]
            pnode = pnodes[i]  # The node is a robot position and 1:1 correspondence
            pnode.setPose(npose)  # Update the position of each node
        self.pcmap.remakeMaps(self.newPoses)  # PointCloudmap modification
