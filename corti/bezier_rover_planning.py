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

def derive_bezier6():
    n = 6
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
    wp_0 = ca.SX.sym('p0', 2, 1)  # pos/vel at waypoint 0
    wp_1 = ca.SX.sym('p1', 2, 1)  # pos/vel at waypoint 1

    constraints = []
    constraints += [(B.eval(0), wp_0[0])]  # pos @ wp0
    constraints += [(B_d.eval(0), wp_0[1])]  # vel @ wp0
    constraints += [(B_d2.eval(0), 0)]  # zero accel @ wp0
    constraints += [(B.eval(T), wp_1[0])]  # pos @ wp1
    constraints += [(B_d.eval(T), wp_1[1])]  # vel @ wp1
    constraints += [(B_d2.eval(T), 0)]  # zero accel @ wp1
    
    assert len(constraints) == 6

    Y = ca.vertcat(*[c[0] for c in constraints])
    b = ca.vertcat(*[c[1] for c in constraints])
    A = ca.jacobian(Y, P)
    A_inv = ca.inv(A)
    P_sol = (A_inv@b).T

    # display(P_sol)
    return {
        'bezier6_solve': ca.Function('bezier6_solve', [wp_0, wp_1, T], [P_sol], ['wp_0', 'wp_1', 'T'], ['P']),
        'bezier6_traj': ca.Function('bezier6_traj', [t, T, P], [r], ['t', 'T', 'P'], ['r']),
        # 'control_points': ca.Function('control_points', [t, T, P], [P], ['t', 'T', 'P'], ['P'])
    }

def rover_plan(bc,T0):
    # T0 = 2.3151252777563114  ##Use optimized time from "dev-Bezier_FindOptimizeFunc.ipynb"
    bezier_6 = derive_bezier6()

    bc = np.array(bc)
    t0 = np.linspace(0, T0, 1000)

    PX = bezier_6['bezier6_solve'](bc[:, 0, 0], bc[:, 1, 0], T0)
    traj_x = np.array(bezier_6['bezier6_traj'](np.array([t0]), T0, PX)).T

    PY = bezier_6['bezier6_solve'](bc[:, 0, 1], bc[:, 1, 1], T0)
    traj_y = np.array(bezier_6['bezier6_traj'](np.array([t0]), T0, PY)).T

    x = traj_x[:, 0]
    vx = traj_x[:, 1]
    ax = traj_x[:, 2]
    jx = traj_x[:,3]
    sx = traj_x[:,4]

    y = traj_y[:, 0]
    vy = traj_y[:, 1]
    ay = traj_y[:, 2]
    jy = traj_y[:, 3]
    sy = traj_y[:, 4]

    V = np.sqrt(vx**2 + vy**2)

    return PX, PY, x,vx,ax,jx,sx, y,vy,ay,jy, sy

def generate_path(bc_t, k):
    
    t_total = 0
    Px1 = np.array([])
    Py1 = np.array([])
    x1 = np.array([])
    vx1 = np.array([])
    ax1 = np.array([])
    jx1 = np.array([])
    sx1 = np.array([])
    y1 = np.array([])
    vy1 = np.array([])
    ay1 = np.array([])
    jy1 = np.array([])
    sy1 = np.array([])


    for i in range(bc_t.shape[1]-1):
        bc = bc_t[:,i:i+2,:]
        T0 = 10
        t_total = t_total + T0 
        Px, Py, x,vx,ax,jx,sx,y,vy,ay,jy,sy= rover_plan(bc,T0)
        Px1 = np.append(Px1, Px)
        Py1 = np.append(Py1, Py)
        x1 = np.append(x1,x)
        vx1 = np.append(vx1,vx)
        ax1 = np.append(ax1,ax)
        jx1 = np.append(jx1,jx)
        sx1 = np.append(sx1,sx)
        y1 = np.append(y1,y)
        vy1 = np.append(vy1,vy)
        ay1 = np.append(ay1,ay)
        jy1 = np.append(jy1,jy)
        sy1 = np.append(sy1,sy)
    t_array = np.linspace(0,t_total,x1.size)

    return Px1, Py1, x1, y1

