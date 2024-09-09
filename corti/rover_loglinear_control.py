import numpy as np
import control
from scipy import signal

"""
This module impelments an SE2 based rover controller.
"""
import math
import numpy as np
from .SE2Lie import *

def solve_control_gain(vr):
    A = -se2(vr, 0, 0).ad_matrix
    B = np.array([[1, 0], [0, 0], [0, 1]])
    Q = 10*np.eye(3)  # penalize state
    R = 1*np.eye(2)  # penalize input
    K, _, _ = control.lqr(A, B, Q, R)  # rescale K, set negative feedback sign
    K = -K
    A0 = B@K
    return B, K, A0 #, A+B@K, B@K

def control_law(B, K, e, case):
    if case =='no_side':
        L = np.diag([1, 0, 1])
    else:
        L = np.eye(3)
    u = L@se2_diff_correction_inv(e)@B@K@e.vee # controller input
    # print(u)
    return u

def maxw(sol, x):
    U1 = np.eye(2)*np.pi/2 # multiply singular val of U
    U2 = np.array([
        [0],
        [0]])
    P = sol['P']
    P1 = P[:2, :]
    P2 = P[2, :]
    mu1 = sol['mu1'] 
    mu2 = sol['mu2']
    alpha = sol['alpha'].real
    
    w1 = (U1.T@P1@x + x.T@P1.T@U1)/(2*alpha*mu1) #disturbance for x y
    w2 = (U2.T@P1 + P2)@x/(alpha*mu2) #disturbance theta
    
    return w1, w2

def compute_control(t, y_vect, ref_data, freq_d, w1_mag, w2_mag, vr, dist, case, use_approx):
    # reference data (from planner, function of time)
    ref_x = ref_data['x']
    ref_y = ref_data['y']
    ref_theta = ref_data['theta']
    ref_omega = ref_data['omega']
    ref_V = ref_data['V']

    # reference value at time t
    r_x = float(ref_x(t))
    r_y = float(ref_y(t))
    r_omega = float(ref_omega(t))
    r_theta = float(ref_theta(t))
    r_V = float(ref_V(t))
    
    # initial states of vehicle and reference and error
    X = SE2(x=y_vect[0], y=y_vect[1], theta=y_vect[2])
    X_r = SE2(x=r_x, y=r_y, theta=r_theta)
    e = se2(x=y_vect[6], y=y_vect[7], theta=y_vect[8]) # log error
    
    # initial error
    eta = X.inv@X_r # error in Lie group
    e_nl = eta.log # error in Lie algebra
    
    B, K, _ = solve_control_gain(vr)
    
    # reference input, coming from planner
    v_r = se2(x=r_V, y=0, theta=r_omega)
    
    # disturbance
    if dist == 'sine':
        phi = 1
        phi2 = 2
        w = se2(x=np.cos(2*np.pi*freq_d*t+phi)*w1_mag, y=np.sin(2*np.pi*freq_d*t+phi)*w1_mag, theta=np.cos(2*np.pi*freq_d*t+phi2)*w2_mag)
    # square wave
    elif dist == 'square':
        w = se2(x=signal.square(2*np.pi*freq_d*t+np.pi)*w1_mag/np.sqrt(2), y=signal.square(2*np.pi*freq_d*t)*w1_mag/np.sqrt(2), theta=signal.square(2*np.pi*freq_d*t)*w2_mag)
    # no disturbance
    else: 
        w = se2(x=0, y=0, theta=0)
        
    # control law applied to non-linear error
    u_nl = se2.from_vector(control_law(B, K, e_nl, case))
    v_nl = v_r + u_nl + w
    
    # control law applied to log-linear error
    u = control_law(B, K, e, case)
    us = se2.from_vector(u)
    v = v_r + us + w
        
    # log error dynamics
    U = se2_diff_correction(e)
    if use_approx:
        # these dynamics don't hold exactly unless you can move sideways
        e_dot = se2.from_vector((-v_r.ad_matrix + B@K)@e.vee + U@w.vee) # does not have L
    else:
        # these dynamics, always hold
        e_dot = -v_r@e + se2.from_vector(U@(us + w).vee) # real control input with L
    
    return [
        # actual nonlinear 
        v_nl.x*np.cos(X.theta) - v_nl.y*np.sin(X.theta),
        v_nl.x*np.sin(X.theta) + v_nl.y*np.cos(X.theta),
        v_nl.theta,
        # reference
        v_r.x*np.cos(X_r.theta) - v_r.y*np.sin(X_r.theta),
        v_r.x*np.sin(X_r.theta) + v_r.y*np.cos(X_r.theta),
        v_r.theta,
        # log error
        e_dot.x,
        e_dot.y,
        e_dot.theta
    ]
