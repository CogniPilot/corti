import casadi as ca
from scipy.special import binom
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from zipfile import ZipFile
import os
import datetime
from .TimeOptBez import *


class Bezier:
#https://en.wikipedia.org/wiki/B%C3%A9zier_curve

    def __init__(self, P: ca.SX, T: float):
        self.P = P
        self.m = P.shape[0]
        self.n = P.shape[1]-1
        self.T = T
    
    def eval(self, t):
        #https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
        beta = t/self.T
        A = ca.SX(self.P)
        for j in range(1, self.n + 1):
            for k in range(self.n + 1 - j):
                A[:, k] = A[:, k] * (1 - beta) + A[:, k + 1] * beta
        return A[:, 0]
    
    def deriv(self, m=1):
        D = ca.SX(self.P)
        for j in range(0, m):
            D = (self.n - j)*ca.horzcat(*[ D[:, i+1] - D[:, i] for i in range(self.n - j) ])
        return Bezier(D/self.T**m, self.T)

def derive_bezier7():
    n = 8
    T = ca.SX.sym('T')
    t = ca.SX.sym('t')
    P = ca.SX.sym('P', 1, n)
    B = Bezier(P, T)

    # derivatives
    B_d = B.deriv()
    B_d2 = B_d.deriv()
    B_d3 = B_d2.deriv()
    B_d4 = B_d3.deriv()

    # boundary conditions

    # trajectory
    p = B.eval(t)
    v = B_d.eval(t)
    a = B_d2.eval(t)
    j = B_d3.eval(t)
    s = B_d4.eval(t)
    r = ca.vertcat(p, v, a, j, s)

    # given position/velocity boundary conditions, solve for bezier points
    wp_0 = ca.SX.sym('p0', 4, 1)  # pos/vel at waypoint 0
    wp_1 = ca.SX.sym('p1', 4, 1)  # pos/vel at waypoint 1

    constraints = []
    constraints += [(B.eval(0), wp_0[0])]  # pos @ wp0
    constraints += [(B_d.eval(0), wp_0[1])]  # vel @ wp0
    constraints += [(B_d2.eval(0), wp_0[2])]  # zero accel @ wp0
    constraints += [(B_d3.eval(T), wp_0[3])]  # zero accel @ wp1
    constraints += [(B.eval(T), wp_1[0])]  # pos @ wp1
    constraints += [(B_d.eval(T), wp_1[1])]  # vel @ wp1
    constraints += [(B_d2.eval(T), wp_1[2])]  # zero accel @ wp1
    constraints += [(B_d3.eval(T), wp_1[3])]  # zero accel @ wp1
    
    assert len(constraints) == n

    Y = ca.vertcat(*[c[0] for c in constraints])
    b = ca.vertcat(*[c[1] for c in constraints])
    A = ca.jacobian(Y, P)
    A_inv = ca.inv(A)
    P_sol = (A_inv@b).T

    return {
        'bezier7_solve': ca.Function('bezier7_solve', [wp_0, wp_1, T], [P_sol], ['wp_0', 'wp_1', 'T'], ['P']),
        'bezier7_traj': ca.Function('bezier7_traj', [t, T, P], [r], ['t', 'T', 'P'], ['r']),
    }

def derive_bezier3():
    n = 4
    T = ca.SX.sym('T')
    t = ca.SX.sym('t')
    P = ca.SX.sym('P', 1, n)
    B = Bezier(P, T)

    # derivatives
    B_d = B.deriv()
    B_d2 = B_d.deriv()
    B_d3 = B_d2.deriv()
    B_d4 = B_d3.deriv()

    # boundary conditions

    # trajectory
    p = B.eval(t)
    v = B_d.eval(t)
    a = B_d2.eval(t)
    r = ca.vertcat(p, v, a)

    # given position/velocity boundary conditions, solve for bezier points
    wp_0 = ca.SX.sym('p0', 2, 1)  # pos/vel at waypoint 0
    wp_1 = ca.SX.sym('p1', 2, 1)  # pos/vel at waypoint 1

    constraints = []
    constraints += [(B.eval(0), wp_0[0])]  # pos @ wp0
    constraints += [(B_d.eval(0), wp_0[1])]  # vel @ wp0
    constraints += [(B.eval(T), wp_1[0])]  # pos @ wp1
    constraints += [(B_d.eval(T), wp_1[1])]  # vel @ wp1

    assert len(constraints) == n

    Y = ca.vertcat(*[c[0] for c in constraints])
    b = ca.vertcat(*[c[1] for c in constraints])
    A = ca.jacobian(Y, P)
    A_inv = ca.inv(A)
    P_sol = (A_inv@b).T

    functions = [
        ca.Function('bezier3_solve', [wp_0, wp_1, T], [P_sol],
            ['wp_0', 'wp_1', 'T'], ['P']),
        ca.Function('bezier3_traj', [t, T, P], [r],
            ['t', 'T', 'P'], ['r']),
    ]

    return { f.name(): f for f in functions }

def multirotor_plan(bc,T0):
    bezier_7 = derive_bezier7()

    bc = np.array(bc)
    t0 = np.linspace(0, T0, 1000)

    PX = bezier_7['bezier7_solve'](bc[:, 0, 0], bc[:, 1, 0], T0)
    traj_x = np.array(bezier_7['bezier7_traj'](np.array([t0]), T0, PX)).T

    PY = bezier_7['bezier7_solve'](bc[:, 0, 1], bc[:, 1, 1], T0)
    traj_y = np.array(bezier_7['bezier7_traj'](np.array([t0]), T0, PY)).T

    V = np.sqrt(vx**2 + vy**2)

    return PX, PY, traj_x, traj_y

