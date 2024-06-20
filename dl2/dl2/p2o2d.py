#!/usr/bin/python
# coding: utf-8
# This Python program is translated by Atsushi Sakai from the following C++ software
#  and slightly modified by Shuro Nakajima:
#  p2o: Petite Portable Pose-graph Optimizer (https://github.com/furo-org/p2o) written by Kiyoshi Irie,
#   Future Robotics Technology Center (fuRo), Chiba Institute of Technology.
# This source code form is subject to the terms of the Mozilla Public License, v. 2.0.
# If a copy of the MPL was not distributed with this file, you can obtain one
#  at https://mozilla.org/MPL/2.0/.

import sys
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse
from scipy.sparse import linalg


class Optimizer2D:
    def __init__(self):
        self.verbose = False
        self.animation = False
        self.p_lambda = 0.0
        self.init_w = 1e10
        self.stop_thre = 1e-3
        self.dim = 3  # state dimension

    def optimize_path(self, nodes, consts, max_iter, min_iter):
        graph_nodes = nodes[:]
        prev_cost = sys.float_info.max
        for i in range(max_iter):
            start = time.time()
            cost, graph_nodes = self.optimize_path_one_step(graph_nodes, consts)
            elapsed = time.time() - start
            if self.verbose:
                print("step ", i, " cost: ", cost, " time:", elapsed, "s")

            # check convergence
            if (i > min_iter) and (prev_cost - cost < self.stop_thre):
                if self.verbose:
                    print("converged:", prev_cost
                          - cost, " < ", self.stop_thre)
                    break
            prev_cost = cost
            if self.animation:
                plt.cla()
                plot_nodes(nodes, color="-b")
                plot_nodes(graph_nodes)
                plt.axis("equal")
                plt.pause(1.0)
        return graph_nodes

    def optimize_path_one_step(self, graph_nodes, constraints):
        indlist = [i for i in range(self.dim)]
        numnodes = len(graph_nodes)
        bf = np.zeros(numnodes * self.dim)
        tripletList = TripletList()
        for con in constraints:
            ida = con.id1
            idb = con.id2
            assert 0 <= ida and ida < numnodes, "ida is invalid"
            assert 0 <= idb and idb < numnodes, "idb is invalid"
            r, Ja, Jb = self.calc_error(
                graph_nodes[ida], graph_nodes[idb], con.t)

            trJaInfo = Ja.transpose() @ con.info_mat
            trJaInfoJa = trJaInfo @ Ja
            trJbInfo = Jb.transpose() @ con.info_mat
            trJbInfoJb = trJbInfo @ Jb
            trJaInfoJb = trJaInfo @ Jb
            for k in indlist:
                for m in indlist:
                    tripletList.push_back(
                        ida * self.dim + k, ida * self.dim + m, trJaInfoJa[k, m])
                    tripletList.push_back(
                        idb * self.dim + k, idb * self.dim + m, trJbInfoJb[k, m])
                    tripletList.push_back(
                        ida * self.dim + k, idb * self.dim + m, trJaInfoJb[k, m])
                    tripletList.push_back(
                        idb * self.dim + k, ida * self.dim + m, trJaInfoJb[m, k])
            bf[ida * self.dim: ida * self.dim + 3] += trJaInfo @ r
            bf[idb * self.dim: idb * self.dim + 3] += trJbInfo @ r
        for k in indlist:
            tripletList.push_back(k, k, self.init_w)
        for i in range(self.dim * numnodes):
            tripletList.push_back(i, i, self.p_lambda)

        mat = sparse.coo_matrix((tripletList.data, (tripletList.row, tripletList.col)),
                                shape=(numnodes * self.dim, numnodes * self.dim))
        x = linalg.spsolve(mat.tocsr(), -bf)
        out_nodes = []
        for i in range(len(graph_nodes)):
            u_i = i * self.dim
            pos = Pose2D_p2o(
                graph_nodes[i].x + x[u_i],
                graph_nodes[i].y + x[u_i + 1],
                graph_nodes[i].theta + x[u_i + 2]
            )
            out_nodes.append(pos)
        cost = self.calc_global_cost(out_nodes, constraints)
        return cost, out_nodes

    def calc_global_cost(self, nodes, constraints):
        cost = 0.0
        for c in constraints:
            diff = self.error_func(nodes[c.id1], nodes[c.id2], c.t)
            cost += diff.transpose() @ c.info_mat @ diff
        return cost

    def error_func(self, pa, pb, t):
        ba = self.calc_constraint_pose(pb, pa)
        error = np.array([ba.x - t.x,
                          ba.y - t.y,
                          self.pi2pi(ba.theta - t.theta)])
        return error

    def calc_constraint_pose(self, _l, r):
        diff = np.array([_l.x - r.x, _l.y - r.y, _l.theta - r.theta])
        v = self.rot_mat_2d(-r.theta) @ diff
        v[2] = self.pi2pi(_l.theta - r.theta)
        return Pose2D_p2o(v[0], v[1], v[2])

    def rot_mat_2d(self, theta):
        sin_theta = math.sin(theta)
        cos_theta = math.cos(theta)
        return np.array([[cos_theta, -sin_theta, 0.0],
                         [sin_theta, cos_theta, 0.0],
                         [0.0, 0.0, 1.0]
                         ])

    def calc_error(self, pa, pb, t):
        e0 = self.error_func(pa, pb, t)
        dx = pb.x - pa.x
        dy = pb.y - pa.y
        sin_theta = math.sin(pa.theta)
        cos_theta = math.cos(pa.theta)
        dxdt = -sin_theta * dx + cos_theta * dy
        dydt = -cos_theta * dx - sin_theta * dy
        Ja = np.array([[-cos_theta, -sin_theta, dxdt],
                       [sin_theta, -cos_theta, dydt],
                       [0.0, 0.0, -1.0]
                       ])
        Jb = np.array([[cos_theta, sin_theta, 0.0],
                       [-sin_theta, cos_theta, 0.0],
                       [0.0, 0.0, 1.0]
                       ])
        return e0, Ja, Jb

    def pi2pi(self, rad):
        val = math.fmod(rad, 2.0 * math.pi)
        if val > math.pi:
            val -= 2.0 * math.pi
        elif val < -math.pi:
            val += 2.0 * math.pi
        return val


class TripletList:
    def __init__(self):
        self.row = []
        self.col = []
        self.data = []

    def push_back(self, irow, icol, idata):
        self.row.append(irow)
        self.col.append(icol)
        self.data.append(idata)


class Pose2D_p2o:
    def __init__(self, x=0., y=0., theta=0.):
        self.x = x
        self.y = y
        self.theta = theta


class Con2D:
    def __init__(self, id1=0, id2=0, t=None, info_mat=None):
        self.id1 = id1
        self.id2 = id2
        self.t = t if t else Pose2D_p2o()
        self.info_mat = info_mat if info_mat else np.eye(3)


def plot_nodes(nodes, color="-r", label=""):
    x, y = [], []
    for n in nodes:
        x.append(n.x)
        y.append(n.y)
    plt.plot(x, y, color, label=label)


def load_data(fname):
    nodes, consts = [], []
    for line in open(fname):
        sline = line.split()
        tag = sline[0]
        if tag == "VERTEX_SE2":
            # data_id = int(sline[1])
            x = float(sline[2])
            y = float(sline[3])
            theta = float(sline[4])
            nodes.append(Pose2D_p2o(x, y, theta))
        elif tag == "EDGE_SE2":
            id1 = int(sline[1])
            id2 = int(sline[2])
            x = float(sline[3])
            y = float(sline[4])
            th = float(sline[5])
            c1 = float(sline[6])
            c2 = float(sline[7])
            c3 = float(sline[8])
            c4 = float(sline[9])
            c5 = float(sline[10])
            c6 = float(sline[11])
            t = Pose2D_p2o(x, y, th)
            info_mat = np.array([[c1, c2, c3],
                                 [c2, c4, c5],
                                 [c3, c5, c6]
                                 ])
            consts.append(Con2D(id1, id2, t, info_mat))
    print("n_nodes:", len(nodes))
    print("n_consts:", len(consts))
    return nodes, consts
