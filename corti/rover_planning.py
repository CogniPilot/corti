from cProfile import label
import os
import numpy as np
import logging
import sys
import matplotlib.pyplot as plt
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
    return PolyTraj(poly_order=poly_order, coeff=coeff, leg_times=leg_times)


def wrap(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


class RoverPlanner:
    
    def __init__(self, x, y, v, theta, r):
        waypoints = []
        velocities = []
        leg_times = []
        
        # started
        pos_stopped = np.array([[x, y]])
        v_start = v
        vel_stopped = np.array([[v_start*np.cos(theta), v_start*np.sin(theta)]])
        waypoints.append(pos_stopped)
        velocities.append(vel_stopped)
        
        # accel
        vel = np.array([[v*np.cos(theta), v*np.sin(theta)]])
        dir_0 = vel/np.linalg.norm(vel)
        pos_accel = pos_stopped + r*dir_0
        d_accel = r 
        v_accel = v
        vel_accel = v_accel*dir_0
        T_accel = d_accel/((v_accel + v_start)/2)
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
    
        def f_ref_V(t): # ref_x(t, 1): first derivative of x
            return np.sqrt(ref_x(t, 1)**2 + ref_y(t, 1)**2)
        
        def f_ref_vx(t):
            return ref_x(t, 1)
        
        def f_ref_vy(t):
            return ref_y(t, 1)
        
        def f_ref_a(t):
            return (ref_x(t, 1)*ref_x(t, 2) + ref_y(t, 1)*ref_y(t, 2))/f_ref_V(t)

        def f_ref_omega(t):
            return (ref_x(t, 1)*ref_y(t, 2) - ref_y(t, 1)*\
                               ref_x(t, 2))/f_ref_V(t)**2
    
        def f_ref_theta(t):
            return np.arctan2(ref_y(t, 1), ref_x(t, 1))

        def f_ref_alpha(t):
            return (ref_y(t, 3)*ref_x(t, 1)**3 + ref_y(t, 1)**2*(2*ref_x(t, 2)*ref_y(t,2) - ref_x(t, 3)*ref_y(t, 1)) +
                    ref_x(t, 1)*ref_y(t, 1)*(2*ref_x(t, 2)**2 - 2*ref_y(t, 2)**2 + ref_y(t, 3)*ref_y(t, 1)) + 
                    ref_x(t, 1)**2*(-ref_x(t, 3)*ref_y(t, 1)-2*ref_x(t, 2)*ref_y(t, 2)))/f_ref_V(t)**4

        if plot:
            t = np.arange(0, np.sum(self.leg_times), 0.1)
            self.px = ref_x(t)
            self.py = ref_y(t)
            plt.figure()
            plt.plot(ref_x(t), ref_y(t))
            plt.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'x', label='waypoints')
            plt.axis('equal')
            plt.title('Planned Trajectory', fontsize=20)
            plt.legend(loc=1, prop={'size': 20})
            plt.grid()
            plt.xlabel('x, m', fontsize=20)
            plt.ylabel('y, m', fontsize=20)

            plt.figure()
            plt.title('V', fontsize=20)
            plt.plot(t, f_ref_V(t))
            plt.grid()
            plt.vlines(np.cumsum(self.leg_times), 0.8, 1.2, color='r', alpha=0.5)
            plt.xlabel('t, sec', fontsize=20)
            plt.ylabel('m/s', fontsize=20)

            plt.figure()
            plt.title('Vx', fontsize=20)
            plt.plot(t, f_ref_vx(t))
            plt.grid()
            plt.vlines(np.cumsum(self.leg_times), 0.8, 1.2, color='r', alpha=0.5)
            plt.xlabel('t, sec', fontsize=20)
            plt.ylabel('m/s', fontsize=20)

            plt.figure()
            plt.title('Vy', fontsize=20)
            plt.plot(t, f_ref_vy(t))
            plt.grid()
            plt.vlines(np.cumsum(self.leg_times), 0.8, 1.2, color='r', alpha=0.5)
            plt.xlabel('t, sec', fontsize=20)
            plt.ylabel('m/s', fontsize=20)

            plt.figure()
            plt.title('$\omega$', fontsize=20)
            plt.plot(t, (f_ref_omega(t)))
            plt.grid()
            plt.xlabel('t, sec', fontsize=20)
            plt.ylabel('rad/s', fontsize=20)
            # plt.savefig('figures/ref_traj_ns.eps', format='eps', bbox_inches='tight')

            plt.figure()
            plt.title('a', fontsize=20)
            plt.plot(t, f_ref_a(t))
            plt.grid()
            # plt.vlines(np.cumsum(self.leg_times), 0.8, 1.2, color='r', alpha=0.5)
            plt.xlabel('t, sec', fontsize=20)
            plt.ylabel('m/s2', fontsize=20)

            plt.figure()
            plt.title('alpha', fontsize=20)
            plt.plot(t, (f_ref_alpha(t)))
            plt.grid()
            # plt.vlines(np.cumsum(self.leg_times), -60, 60, color='r', alpha=0.5)
            plt.xlabel('t, sec', fontsize=20)
            plt.ylabel('rad/s2', fontsize=20)

            plt.figure()
            plt.title('theta')
            plt.plot(t, (f_ref_theta(t)))
            plt.grid()
            plt.xlabel('t, sec')
            plt.ylabel('rad')
            plt.vlines(np.cumsum(self.leg_times), -180, 180, color='r', alpha=0.5)

        return {
            'x': f_ref_x,
            'y': f_ref_y,
            'V': f_ref_V,
            'vx': f_ref_vx,
            'vy': f_ref_vy,
            'theta': f_ref_theta,
            'omega': f_ref_omega,
            'poly_x': ref_x,
            'poly_y': ref_y,
            'way_points': np.array([self.px, self.py]),
            't': np.arange(0, np.sum(self.leg_times), 0.1)
        }
