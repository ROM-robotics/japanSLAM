#!/usr/bin/python
# coding: utf-8

import numpy as np

from my_util import DEG2RAD, RAD2DEG
from pose2d import Pose2D
from pose_graph import PoseGraph
from p2o2d import Pose2D_p2o, Con2D, Optimizer2D

# Start the pose graph optimized library KSLAM.
class P2oDriver2D:
    def __init__(self):
        pass

    def doP2o(self, pg=None, newPoses=None, N=0):
        pg = pg if pg else PoseGraph()
        newPoses = newPoses if newPoses else np.empty(0)
        nodes = pg.nodes  # Pose node
        arcs = pg.arcs  # Pose arc

        # Convert pose node to P2O
        pnodes = np.empty(0)  # P2O pose node set
        len_nodes = len(nodes)
        np_append = np.append
        for num in range(0, len_nodes, 1):
            node = nodes[num]
            pose = node.pose  # Node position
            pnodes = np_append(pnodes, Pose2D_p2o(pose.tx, pose.ty, DEG2RAD(pose.th)))  # Just put in the position
        # Convert pose arc to KSLAM
        pcons = np.empty(0)  # P2O pose arc set
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
            pcons = np_append(pcons, con)  # Just put in the position

        opt = Optimizer2D()  # P2O instance
        result = opt.optimize_path(pnodes, pcons, N, 3)  # N -time execution

        # Store the result in Newpose
        for num in range(len(result)):
            newPose = result[num]  # Corrected position of II node
            pose = Pose2D(newPose.x, newPose.y, RAD2DEG(newPose.theta))
            newPoses = np_append(newPoses, pose)
        return newPoses
