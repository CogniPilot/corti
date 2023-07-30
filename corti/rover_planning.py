from cProfile import label
import os
import numpy as np
import logging
import sys
import sympy
from sympy import symbols, factorial, Function, summation, binomial, product
import matplotlib.pyplot as plt
from .rover_control import *
from .SE2Lie import *
"""
This module rover minimum jerk trajectory planning.
"""

class PolyTraj:
    
    def __init__(self, poly_order, coeff, leg_times):
        assert len(leg_times) == len(coeff)/(poly_order + 1)
        self.poly_order = poly_order
        self.leg_times = leg_times
        self.poly_leg = [
            np.polynomial.Polynomial(coeff[j*(poly_order+1):(j+1)*(poly_order+1)])
            for j in range(len(leg_times))
        ]
        self.t_start_leg = np.cumsum(np.hstack([0, self.leg_times]))

    
    def __call__(self, t, m=0):
        t = np.atleast_1d(t)
        t_start = t[np.argwhere(t < 0)]
        vals = self.poly_leg[0].deriv(m)(0)*np.ones(len(t_start))/self.leg_times[0]**m
        for i_leg in range(len(self.leg_times)):
            t_i = t[np.argwhere(np.logical_and(t >= self.t_start_leg[i_leg], t < self.t_start_leg[i_leg+1]))]
            if len(t_i) == 0:
                continue
            beta = (t_i - self.t_start_leg[i_leg])/self.leg_times[i_leg]
            vals = np.hstack([vals, self.poly_leg[i_leg].deriv(m)(beta)[:, 0]/self.leg_times[i_leg]**m])
        t_end = t[np.argwhere(t >= self.t_start_leg[len(self.leg_times)])]
        vals = np.hstack([vals, self.poly_leg[-1].deriv(m)(1)*np.ones(len(t_end))/self.leg_times[-1]**m])
        return vals

n, i, j, m = symbols('n, i, j, m', integer=True, real=True)

t = symbols('t')
P = Function('P')

C = sympy.factorial(n)/sympy.factorial(n - j)* summation((-1)**(i + j)*P(i)/(factorial(i)*factorial(j - i)), (i, 0, j)) #use diffe
    
n0 = 7

P_vect = sympy.Matrix([P(i) for i in range(n0)])
    
C_matrix = sympy.Matrix([C.subs({j: j0, n: n0}).doit() for j0 in range(n0)])

A = C_matrix.jacobian(P_vect)

C_to_B = np.array(A.inv(), dtype=float)


def plan_trajectory_1d(poly_order, waypoints, velocities, leg_times, continuity_derivs):

    def constraint_eq(poly_order, derivative, scaled_time, leg_time, i_leg, n_legs):
        n = poly_order
        m = derivative
        beta = scaled_time
        T = leg_time
        a = np.zeros(n_legs*(n+1))
        for k in range(m, n+1):
            l = 1
            for j in range(k, k-m, -1):
                l *= j
            #assert l == np.math.factorial(k)/np.math.factorial(k-m)
            a[i_leg*(n+1) + k] = l*beta**(k-m)/T**m
        return a

    waypoints = np.atleast_1d(waypoints)
    velocities = np.atleast_1d(velocities)

    A = []
    b = []
    n_legs = len(leg_times)
    assert len(leg_times) == len(waypoints) - 1
    
    n_constraints = 0
    
    # waypoints
    for i_leg in range(len(leg_times)):  

        # start
        logging.info('leg %d start ', i_leg)
        A.append(constraint_eq(poly_order=poly_order, derivative=0, scaled_time=0,
                               leg_time=leg_times[i_leg], i_leg=i_leg, n_legs=n_legs))
        b.append(waypoints[i_leg])
        n_constraints += 1

        # end
        logging.info('leg %d end', i_leg)
        A.append(constraint_eq(poly_order=poly_order, derivative=0, scaled_time=1,
                               leg_time=leg_times[i_leg], i_leg=i_leg, n_legs=n_legs))
        b.append(waypoints[i_leg + 1])
        n_constraints += 1
       
        for m in [1, 2]:
            # velocity start
            logging.info('leg %d velocity start', i_leg)
            A.append(constraint_eq(poly_order=poly_order, derivative=m, scaled_time=0,
                                   leg_time=leg_times[i_leg], i_leg=i_leg, n_legs=n_legs))
            if m == 1:
                b.append(velocities[i_leg])
            else:
                b.append(0)
            n_constraints += 1

            # velocity end
            logging.info('leg %d end', i_leg)
            A.append(constraint_eq(poly_order=poly_order, derivative=m, scaled_time=1,
                                   leg_time=leg_times[i_leg], i_leg=i_leg, n_legs=n_legs))
            if m == 1:
                b.append(velocities[i_leg + 1])
            else:
                b.append(0)
            n_constraints += 1 


    # continuity
    for m in continuity_derivs:
        for i_leg in range(len(leg_times) - 1):
            # start
            logging.info('leg %d continuity %d', i_leg, m)
            A.append(constraint_eq(poly_order=poly_order, derivative=m, scaled_time=1,
                                   leg_time=leg_times[i_leg], i_leg=i_leg, n_legs=n_legs)
                     - constraint_eq(poly_order=poly_order, derivative=m, scaled_time=0,
                            leg_time=leg_times[i_leg + 1], i_leg=i_leg + 1, n_legs=n_legs)
                    )
            b.append(0)
            n_constraints += 1

    A = np.array(A)
    b = np.array(b)

    logging.info('n_constraints: %d', n_constraints)
    logging.info('n_coeff: %d', poly_order + 1)
    
    assert n_constraints <= n_legs*(poly_order + 1)
    
    coeff = np.linalg.pinv(A).dot(b)
    b_coeff = np.zeros(coeff.shape)
    for n in range(n_legs):
        b_coeff[n*(poly_order+1):(n+1)*(poly_order+1)] = C_to_B@coeff[n*(poly_order+1):(n+1)*(poly_order+1)]
    return PolyTraj(poly_order=poly_order, coeff=b_coeff, leg_times=leg_times)

def wrap(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


class RoverPlanner:
    
    def __init__(self, x, y, v, theta, r):
        waypoints = []
        velocities = []
        leg_times = []
        
        # started
        pos_stopped = np.array([[x, y]])
        vel_stopped = np.array([[0, 0]])
        waypoints.append(pos_stopped)
        velocities.append(vel_stopped)
        
        # accel
        vel = np.array([[v*np.cos(theta), v*np.sin(theta)]])
        dir_0 = vel/np.linalg.norm(vel)
        pos_accel = pos_stopped + r*dir_0
        d_accel = r 
        v_accel = v
        vel_accel = v_accel*dir_0
        T_accel = d_accel/((v_accel + 0)/2)
        waypoints.append(pos_accel)
        velocities.append(vel_accel)
        leg_times.append(T_accel)
        
        self.waypoints = np.vstack(waypoints)
        self.velocities = np.vstack(velocities)
        self.leg_times = np.array(leg_times)
        self.px = []
        self.py = []

    def stop(self, x, y):
        pos = np.array([x, y])
        vel_prev = self.velocities[-1] 
        v = np.linalg.norm(vel_prev)
        pos_prev = self.waypoints[-1]
        delta_pos = pos - pos_prev
        d = np.linalg.norm(delta_pos)
        T = d/(v*0.5)

        pos_stopped = np.array([[x, y]])
        vel_stopped = np.array([[0, 0]])
        
        self.waypoints = np.vstack([self.waypoints, pos_stopped])
        self.velocities = np.vstack([self.velocities, vel_stopped])
        self.leg_times = np.hstack([self.leg_times, T])
        
    def goto(self, x, y, v, r):
        pos = np.array([x, y])
        pos_prev = self.waypoints[-1]
        vel_prev = self.velocities[-1]
        v_prev = np.linalg.norm(vel_prev)
        
        # old heading
        speed0 = max([1e-3, np.linalg.norm(vel_prev)])
        dir_0 = vel_prev/speed0
        theta0 = np.arctan2(vel_prev[1], vel_prev[0])

        # new heading
        delta_pos = pos - pos_prev - r*dir_0
        dir_1 = delta_pos/np.linalg.norm(delta_pos)
        theta1 = np.arctan2(delta_pos[1], delta_pos[0])
        
        waypoints = []
        velocities = []
        leg_times = []
        
        # turn
        dtheta = np.abs(wrap(theta1 - theta0))
        if dtheta > 0:
            pos_turn = pos_prev + r*dir_0 + r*dir_1
            v_turn = v
            vel_turn = v_turn*dir_1
            d_turn = r*dtheta
            T_turn = d_turn/v_turn*1.1
        
            waypoints.append(pos_turn)
            velocities.append(vel_turn)
            leg_times.append(T_turn)
        
        # straight away
        pos_straight = pos - r*dir_1
        if len(waypoints) == 0:
            d_straight = np.linalg.norm(pos_straight - pos_prev)
        else:
            d_straight = np.linalg.norm(pos_straight - waypoints[-1])
        v_straight = v
        vel_straight = v_straight*dir_1
        T_straight = d_straight/v_straight
        
        waypoints.append(pos_straight)
        velocities.append(vel_straight)
        leg_times.append(T_straight)
        
        self.waypoints = np.vstack([self.waypoints, waypoints])
        self.velocities = np.vstack([self.velocities, velocities])
        self.leg_times = np.hstack([self.leg_times, leg_times])


    def compute_ref_data(self, plot=False):
        poly_order = 6
        ref_x = plan_trajectory_1d(poly_order=poly_order,
                                waypoints=self.waypoints[:, 0],
                                velocities=self.velocities[:, 0],
                                leg_times=self.leg_times,
                                continuity_derivs=[])
        ref_y = plan_trajectory_1d(poly_order=poly_order,
                                waypoints=self.waypoints[:, 1],
                                velocities=self.velocities[:, 1],
                                leg_times=self.leg_times,
                                continuity_derivs=[])
        
        def f_ref_x(t):
            return ref_x(t)
        
        def f_ref_y(t):
            return ref_y(t)
    
        def f_ref_V(t):
            return np.sqrt(ref_x(t, 1)**2 + ref_y(t, 1)**2)
        
        def f_ref_omega(t):
            return (ref_x(t, 1)*ref_y(t, 2) - ref_y(t, 1)*\
                               ref_x(t, 2))/f_ref_V(t)**2
    
        def f_ref_theta(t):
            return np.arctan2(ref_y(t, 1), ref_x(t, 1))

        if plot:
            t = np.arange(0, np.sum(self.leg_times), 0.05)
            plt.rcParams.update({'font.size': 20})
            plt.figure(figsize=(16,24))
            ax = plt.subplot(311)
            ax2 = plt.subplot(312)
            ax3 = plt.subplot(313)
            # plt.figure(dpi=800)
            self.px = ref_x(t)
            self.py = ref_y(t)
            ax.plot(ref_x(t), ref_y(t))
            ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'x', label='waypoints')
            ax.axis('equal')
            ax.set_title('Planned Trajectory', fontsize=20)
            ax.legend(loc=1, prop={'size': 20})
            ax.grid()
            ax.set_xlabel('x, m', fontsize=20)
            ax.set_ylabel('y, m', fontsize=20)
            # plt.savefig('ref_traj1.png', dpi=2500)

            # plt.figure(dpi=800)
            ax2.set_title('V', fontsize=20)
            ax2.plot(t, f_ref_V(t))
            ax2.grid()
            ax2.vlines(np.cumsum(self.leg_times), 0.8, 1.2, color='r', alpha=0.5)
            ax2.set_xlabel('t, sec', fontsize=20)
            ax2.set_ylabel('m/s', fontsize=20)
            # plt.savefig('ref_traj2.png', dpi=2500)

            # plt.figure(dpi=800)
            ax3.set_title('$\omega$', fontsize=20)
            ax3.plot(t, np.rad2deg(f_ref_omega(t)))
            ax3.grid()
            ax3.vlines(np.cumsum(self.leg_times), -60, 60, color='r', alpha=0.5)
            ax3.set_xlabel('t, sec', fontsize=20)
            ax3.set_ylabel('deg/s', fontsize=20)

            # plt.savefig('figures/ref_traj_ns.eps', format='eps', bbox_inches='tight')

            # plt.savefig('ref_traj3.png', dpi=500)

            plt.figure()
            plt.title('theta')
            plt.plot(t, np.rad2deg(f_ref_theta(t)))
            plt.grid()
            plt.xlabel('t, sec')
            plt.ylabel('deg')
            plt.vlines(np.cumsum(self.leg_times), -180, 180, color='r', alpha=0.5)

        return {
            'x': f_ref_x,
            'y': f_ref_y,
            'V': f_ref_V,
            'theta': f_ref_theta,
            'omega': f_ref_omega,
            'poly_x': ref_x,
            'poly_y': ref_y,
            'way_points': np.array([self.px, self.py]),
            't': np.arange(0, np.sum(self.leg_times), 0.05),
        }

def simulate_rover(planner: RoverPlanner, freq_d, w1, w2, x0, y0, theta0, vr, dist, case, use_approx, dt, plot=False):
    t = np.arange(0, np.sum(planner.leg_times), dt)
    ref_data = planner.compute_ref_data()
    X0 = SE2(x=x0, y=y0, theta=theta0)  # initial state in SE2
    X0_r = SE2(x=0, y=0, theta=0)  # initial reference state
    x0 = (X0.inv@X0_r).log  # initial log of error
    import scipy.integrate
#     res = scipy.integrate.solve_ivp(
#         fun=lambda t, x_vect: kinematics(t, x_vect, ref_data, freq_d, w1, w2, dist, sol, use_approx),
#             t_span=[t[0], t[-1]], y0=[X0.x, X0.y, X0.theta], t_eval=t)
    res = scipy.integrate.solve_ivp(
        fun=compute_control,
        t_span=[t[0], t[-1]], t_eval=t,
        y0=[X0.x, X0.y, X0.theta,
            X0_r.x, X0_r.y, X0_r.theta,
            x0.x, x0.y, x0.theta], args=[ref_data, freq_d, w1, w2, vr, dist, case, use_approx], rtol=1e-6, atol=1e-9)
    return res

def compute_exp_log_err(e_x, e_y, e_theta, x_r, y_r, theta_r):
    return (SE2(x=x_r, y=y_r, theta=theta_r)@((se2(x=e_x, y=e_y, theta=e_theta).exp).inv)).params


def compute_err(x, y, theta, x_r, y_r, theta_r):
    return (SE2(x=x, y=y, theta=theta).inv@SE2(x=x_r, y=y_r, theta=theta_r)).log.vee

def compute_log_err(ex, ey, etheta):
    return (se2(ex, ey, etheta).exp).params

def control_law1(B, K, e):
    L = np.diag([1, 1, 1])
    print(L@se2_diff_correction_inv(e))
    print(B)
    u = L@se2_diff_correction_inv(e)@B@K@e.vee # controller input
    return u

def plot_rover_sim(res, vr, case, planner):
    ref_data = planner.compute_ref_data()
    t = res['t']

    # reference data
    ref_x = ref_data['x']
    ref_y = ref_data['y']
    ref_theta = ref_data['theta']
    ref_omega = ref_data['omega']
    ref_V = ref_data['V']

    # reference at time t
    r_x = ref_x(t)
    r_y = ref_y(t)
    r_theta = ref_theta(t)
    r_omega = ref_omega(t).reshape(len(t),1)
    r_V = ref_V(t).reshape(len(t),1)
    
    y_vect = res['y']
    x, y, theta, x_r, y_r, theta_r, log_e_x, log_e_y, log_e_theta = [y_vect[i, :] for i in range(len(y_vect))]
    
    exp_log_err = np.zeros((3,len(t)))
    for j in range(len(t)):
        exp_log_err[:,j] = np.array([compute_exp_log_err(log_e_x[j], log_e_y[j], log_e_theta[j], r_x[j], r_y[j], r_theta[j])])
    
    # exp_log_err = np.array([compute_exp_log_err(y[6], y[7], y[8], y[3], y[4], y[5]) for y in y_vect.T]).T
    err = np.array([compute_err(y[0], y[1], y[2], y[3], y[4], y[5]) for y in y_vect.T]).T
    log_err = np.array([compute_log_err(y[6], y[7], y[8])for y in y_vect.T]).T
    
    ux = np.zeros((log_e_x.shape[0],1))
    uy = np.zeros((log_e_x.shape[0],1))
    utheta = np.zeros((log_e_x.shape[0],1))
    uw = np.zeros((log_e_x.shape[0],1))
    B, K = solve_control_gain(vr)
    for i in range(log_e_x.shape[0]):
        #print(se2.from_vector(np.array([log_e_x[i], log_e_y[i], log_e_theta[i]])))
        e = se2(err[0,i], err[1,i], err[2,i])
        #print(control_law(B, K, e))
        uc = np.array([control_law(B, K, e, case)])
        ux[i,:] = uc[:,0]
        uy[i,:] = uc[:,1]
#         if np.abs(r_omega[i,:]) > np.pi/4:
#             if r_omega[i,:] < 0:
#                 r_omega[i,:] = -np.pi/4
#             else:
#                 r_omega[i,:] = np.pi/4
        utheta[i,:] = uc[:,2]
        uw[i,:] = np.sqrt(ux[i,:]**2+uy[i,:]**2)
    
    plt.figure()
    plt.plot(exp_log_err[0, :], exp_log_err[1, :], '-')
    # plt.plot(x, y)
    plt.plot(x_r, y_r)
    plt.plot(ref_x(t), ref_y(t))
    plt.xlabel('x, m')
    plt.ylabel('y, m')
    plt.axis('equal')
    plt.grid()
    plt.title('trajectory')

    plt.figure()
    plt.plot(t, log_err[0,:], label='x')
    plt.plot(t, log_err[1,:], label='y')
    plt.legend()
    plt.title('position error')
    plt.grid()
    plt.xlabel('t, sec')
    plt.ylabel('m')

    plt.figure()
    plt.plot(t, log_err[2,:], label='$\\theta$')
    plt.grid()
    plt.legend()
    plt.title('heading error')
    plt.xlabel('t, sec')
    plt.ylabel('rad')
    
    plt.figure()
    plt.plot(t, (ux), label='$u_x$ (m/s)')
    plt.plot(t, (uy), label='$u_y$ (m/s)')
    plt.plot(t, (utheta), label='$u_\omega$ (rad/s)')
    plt.legend(loc=1)
    plt.grid()
    plt.title('Vehicle Inputs')
    plt.xlabel('t, sec')
    plt.ylabel('Value')

    return err

def plot_rover_simulated(res, planner, name=None, legend=False, save=False, **plt_kwargs):
    if save:
        os.makedirs('figures', exist_ok=True)
    
    ref_data = planner.compute_ref_data()
    t = res['t']
    
    ref_x = ref_data['x']
    ref_y = ref_data['y']
    ref_theta = ref_data['theta']
    
    r_x = ref_x(t)
    r_y = ref_y(t)
    r_theta = ref_theta(t)
    
    y_vect = res['y']
    x, y, theta, x_r, y_r, theta_r, log_e_x, log_e_y, log_e_theta = [y_vect[i, :] for i in range(len(y_vect))]    
    exp_log_err = np.zeros((3,len(t)))
    for j in range(len(t)):
        exp_log_err[:,j] = np.array([compute_exp_log_err(log_e_x[j], log_e_y[j], log_e_theta[j], r_x[j], r_y[j], r_theta[j])])
    
    # plot simulated trajectory
    plt.rcParams.update({'font.size': 20})
    plt.figure(1, figsize=(18,12))
    label = 'Simulated Trajectory' + name
    plt.grid(True)
    plt.plot(exp_log_err[0, :], exp_log_err[1, :], label=label  if legend else None, **plt_kwargs)
    plt.xlabel('x, m')
    plt.ylabel('y, m')
    
    if legend:
        plt.legend(loc=1)
    plt.axis('equal')
    if save:
        plt.savefig('figures/')
        
def plot_sim_corres(res, planner, name=None, legend=False, save=False):
    if save:
        os.makedirs('figures', exist_ok=True)
    ref_data = planner.compute_ref_data()
    t = res['t']

    # reference data
    ref_x = ref_data['x']
    ref_y = ref_data['y']
    ref_theta = ref_data['theta']
    ref_omega = ref_data['omega']
    ref_V = ref_data['V']

    # reference at time t
    r_x = ref_x(t)
    r_y = ref_y(t)
    r_theta = ref_theta(t)
    r_omega = ref_omega(t).reshape(len(t),1)
    r_V = ref_V(t).reshape(len(t),1)
    
    y_vect = res['y']
    x, y, theta, x_r, y_r, theta_r, log_e_x, log_e_y, log_e_theta = [y_vect[i, :] for i in range(len(y_vect))]
    
    exp_log_err = np.zeros((3,len(t)))
    for j in range(len(t)):
        exp_log_err[:,j] = np.array([compute_exp_log_err(log_e_x[j], log_e_y[j], log_e_theta[j], r_x[j], r_y[j], r_theta[j])]) # Lie algebra
    
    # exp_log_err = np.array([compute_exp_log_err(y[6], y[7], y[8], y[3], y[4], y[5]) for y in y_vect.T]).T
    err = np.array([compute_err(y[0], y[1], y[2], y[3], y[4], y[5]) for y in y_vect.T]).T
    
    plt.rcParams['figure.figsize'] = (16, 10)
    plt.rcParams.update({'font.size': 20})
    fig1 = plt.figure(1)
    title = 'X-Y Trajectory'
    # plt.plot(exp_log_err[0, :], exp_log_err[1, :], '-')    
    plt.plot(x, y, 'b-.', label='Lie Group' if legend else None, linewidth=4, alpha=1)
    plt.plot(exp_log_err[0, :], exp_log_err[1, :], 'r-', label='Lie Algebra' if legend else None, linewidth=1, alpha = 1)
    plt.plot(ref_x(t), ref_y(t), 'y-', alpha=1, linewidth=3, label='Reference' if legend else None)
    plt.xlabel('x, m')
    plt.ylabel('y, m')
    plt.axis('equal')
    plt.grid(True)
    plt.title(title)
    if legend:
        plt.legend(loc=1)
    if save:
        plt.savefig('Lie_corres.eps', format='eps')
    
    plt.rcParams['figure.figsize'] = (16, 8)
    fig2 = plt.figure(2)
    title = name + 'Error between the Lie Group and the Lie Algebra'
    plt.title(title)
    plt.grid(True)
    plt.plot(t, x-exp_log_err[0, :], 'y--', label='x(m)' if legend else None, linewidth=1, alpha = 1)
    plt.plot(t, y-exp_log_err[1, :], 'b-.', label='y(m)' if legend else None, linewidth=1, alpha = 1)
    theta_err = theta-exp_log_err[2, :]
    for i in range(len(theta_err)):
        if theta_err[i] > 1:
            theta_err[i] = theta_err[i] - 2*np.pi
    plt.plot(t, theta_err, 'r-', label='$\\theta$(rad)' if legend else None, linewidth=1)
    plt.xlabel('t, sec')
    plt.ylabel('error')
    plt.xlim(0,t[-1])
    if legend:
        plt.legend(loc=1)
    if save:
        plt.savefig('Lie_error.eps', format='eps')

    return fig1, fig2, err
