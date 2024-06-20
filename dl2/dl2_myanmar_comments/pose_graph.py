#!/usr/bin/python
# coding: utf-8

import numpy as np

from pose2d import Pose2D

# Pose -graph vertex
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


# Pose -graph side
class PoseArc:
    def __init__(self, src=None, dst=None, relPose=None, inf=None):
        self.src = src  # Node on the starting point of this arc
        self.dst = dst  # Node on the end point of this arc
        self.relPose = relPose if relPose else Pose2D()  # Relative position of this arc(Measured value)
        self.inf = inf if inf else np.eye(3)  # Intelligence

    def setup(self, s, d, rel, inf):
        self.src = s
        self.dst = d
        self.relPose = rel
        self.inf = inf


# Pose graph
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

    # Node generation
    def allocNode(self):
        if len(self.nodePool) >= PoseGraph.POOL_SIZE:
            print("Error: exceeds nodePool capacity %d" % len(self.nodePool))
            return None
        node = PoseNode()
        self.nodePool = np.append(self.nodePool, node)  # Add to memory pool, See it.
        return self.nodePool[-1]

    # Arc generation
    def allocArc(self):
        if len(self.arcPool) >= PoseGraph.POOL_SIZE:
            print("Error: exceeds arcPool capacity")
            return None
        arc = PoseArc()
        self.arcPool = np.append(self.arcPool, arc)  # Add to memory pool, See it.
        return self.arcPool[-1]

    # Graph generation
    # Add node to pose graph
    def addNode(self, pose):
        n1 = self.allocNode()  # Node generation
        self.addNode2(n1, pose)  # Add node to pose graph
        return n1

    # Add node to pose graph
    def addNode2(self, n1, pose):
        n1.setNid(len(self.nodes))  # Node ID gave. Same as the node number
        n1.setPose(pose)  # Set the robot position
        self.nodes = np.append(self.nodes, n1)  # Added to the end of nodes

    # Node ID(nid)Find a node entity from
    def findNode(self, nid):
        for i in range(len(self.nodes)):  # Simply searching for linear
            n = self.nodes[i]
            if n.nid == nid:  # I found it when the NID matched
                return n
        return None

    # Add an arc to the pose graph
    def addArc(self, arc):
        arc.src.addArc(arc)  # Added ARC to the terminal node
        arc.dst.addArc(arc)  # Added ARC to the terminal node
        self.arcs = np.append(self.arcs, arc)  # Add ARC to the end of ARCS

    # Generate an arc between the starting point Node SRCNID and the terminal node DSTNID
    def makeArc(self, srcNid, dstNid, relPose, cov):
        inf = np.linalg.inv(cov)  # INF is the reverse matrix of COV
        src = self.nodes[srcNid]  # Starting Node
        dst = self.nodes[dstNid]  # End point node
        arc = self.allocArc()  # Arc generation
        arc.setup(src, dst, relPose, inf)  # Relpose is a relative position due to measurement
        return arc

    # Returns the arc that the starting point node is SRCNID and the terminal node is DSTNID
    def findArc(self, srcNid, dstNid):
        for i in range(len(self.arcs)):
            a = self.arcs[i]
            if a.src.nid == srcNid and a.dst.nid == dstNid:
                return a
        return None
