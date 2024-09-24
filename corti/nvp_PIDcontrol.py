import numpy as np

"""
This module impelments an SE2 based rover controler.
"""

def wrap(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


class SE2:
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def to_matrix(self):
        x = self.x
        y = self.y
        theta = self.theta
        cos = np.cos
        sin = np.sin
        return np.array([
            [cos(theta), -sin(theta), x],
            [sin(theta), cos(theta), y],
            [0, 0, 1]])

    @classmethod
    def from_matrix(cls, m):
        theta = np.arctan2(m[1, 0], m[0, 0])
        x = m[0, 2]
        y = m[1, 2]
        return cls(x=x, y=y, theta=theta)

    def __matmul__(self, other):
        return SE2.from_matrix(self.to_matrix()@other.to_matrix())

    def __repr__(self):
        return 'x {:g}: y: {:g} theta: {:g}'.format(self.x, self.y, self.theta)
    
    def log(self):
        x = self.x
        y = self.y
        theta = self.theta
        if (np.abs(theta) > 1e-2):
            a = np.sin(theta)/theta
            b = (1 - np.cos(theta))/theta
        else:
            a = 1 - theta**2/6 + theta**4/120
            b = theta/2 - theta**3/24 + theta**5/720
        V_inv = np.array([
            [a, b],
            [-b, a]])/(a**2 + b**2)
        u = V_inv@np.array([x, y])
        return SE2(x=u[0], y=u[1], theta=theta)
    
    def inv(self):
        x = self.x
        y = self.y
        theta = self.theta
        t = -np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]])@np.array([x, y])
        return SE2(x=t[0], y=t[1], theta=-theta)


class se2:
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def to_matrix(self):
        x = self.x
        y = self.y
        theta = self.theta
        return np.array([
            [0, -theta, x],
            [theta, 0, y],
            [0, 0, 0]])

    @classmethod
    def from_matrix(cls, m):
        x = m[0, 2]
        y = m[1, 2]
        theta = m[1, 0]
        return cls(x=x, y=y, theta=theta)

    def __repr__(self):
        return 'x {:g}: y: {:g} theta: {:g}'.format(self.x, self.y, self.theta)
    
    def exp(self):
        x = self.x
        y = self.y
        theta = self.theta
        if (np.abs(theta) > 1e-2):
            a = np.sin(theta)/theta
            b = (1 - np.cos(theta))/theta
        else:
            a = 1 - theta**2/6 + theta**4/120
            b = theta/2 - theta**3/24 + theta**5/720
        V = np.array([
            [a, -b],
            [b, a]])
        u = V@np.array([x, y])
        return SE2(x=u[0], y=u[1], theta=theta)

class PIDControl:
    def __init__(self, dt, args) -> None:
        self.prev_x = 0
        self.prev_y = 0
        self.prev_theta = 0
        self.dt = dt
        self.args = args
        self.error_v_integral = 0
        self.error_v_last = 0
        self.error_omega_integral = 0
        self.error_omega_last = 0
        self.chi = SE2(x=0, y=0, theta=0)

    def compute_control(self, idx, x, y, theta, ref_data, v_est, omega_est):
        # ref data in function of time
        ref_x = ref_data['x']
        ref_y = ref_data['y']
        ref_theta = ref_data['theta']
        ref_omega = ref_data['omega']
        ref_V = ref_data['V']

        # getting point at time t
        r_x = float(ref_x[idx])
        r_y = float(ref_y[idx])
        r_omega = float(ref_omega[idx])
        r_theta = float(ref_theta[idx])
        r_V = float(ref_V[idx])

        # getting the error between state and ref, x, y, theta
        X_r = SE2(r_x, r_y, r_theta)
        X = SE2(x, y, theta)
        eta = X_r.inv()@X
        self.chi = eta.log()

        # control parameters
        K_x = 5.0  # makes vehicle speed up to elim. along track error
        # K_y_to_theta = 1.0  # makes vehicle turn to elim. cross track error
        K_theta = 4.0  # makes vehicle rotate faster for given theta error

        # nvp1
        if self.args==1:
            K_th = 0.04 # high, faster, low, slower 0.1
            K_thi = 0.075
            K_thd = 0
            K_delta = 1.2
            K_deltai = 0.15
            K_deltad = 0
            throttle_integral_max = 0.2
            ref_th = 0.17
            K_y_to_theta = 0.5

        # nvp2
        if self.args==2:
            K_th = 0.01 # high, faster, low, slower 0.1
            K_thi = 0.1
            K_thd = 0
            K_delta = 1.4
            K_deltai = 0.15
            K_deltad = 0
            throttle_integral_max = 0.2
            ref_th = 0.19
            K_y_to_theta = 1.0

        omega_max = np.deg2rad(180)  # deg/s max rotation rate saturation
        y_to_theta_max = np.deg2rad(90)  # y to theta_max saturation, 90 deg, perpin. to track

        # PID control
        # forward vel
        v = r_V - K_x*np.real(self.chi.x)
        # saturation for forward vel
        if v < 0:
            v = 0
        if v > 0.5:
            v = 0.5


        y_to_theta = K_y_to_theta*np.real(self.chi.y)
        if np.abs(y_to_theta > y_to_theta_max):
            y_to_theta = np.sign(y_to_theta)*y_to_theta_max
        # omega
        omega = r_omega - K_theta*(np.real(self.chi.theta) + y_to_theta)
        # saturation for omega
        if abs(omega) > omega_max:
            omega = np.sign(omega)*omega_max
        # print(r_theta, theta, r_omega, omega)
        # omega_est = (self.prev_theta - theta)/self.dt

        # Compute control commands for throttle and rudder
        error_v = v - v_est
        error_v_deriv = (error_v - self.error_v_last)/self.dt
        self.error_v_last = error_v
        self.error_v_integral += error_v*self.dt
        if self.error_v_integral > throttle_integral_max:
            self.error_v_integral = throttle_integral_max
        elif self.error_v_integral < -throttle_integral_max:
            self.error_v_integral = -throttle_integral_max
        throttle = ref_th + K_th*(error_v) + K_thi * self.error_v_integral + K_thd * error_v_deriv
        # print(r_V, v)
        # print(throttle)
        # Saturation for throttle
        if throttle < 0:
            throttle = 0
        if throttle > 1:
            throttle = 1

        delta_integral_max = 0.2
        error_omega = omega - omega_est
        error_omega_deriv = (error_omega - self.error_omega_last)/self.dt
        self.error_omega_last = error_omega
        self.error_omega_integral += error_omega*self.dt
        if self.error_omega_integral > delta_integral_max:
            self.error_omega_integral = delta_integral_max
        elif self.error_omega_integral < -delta_integral_max:
            self.error_omega_integral = -delta_integral_max
        
        delta  = K_delta*(error_omega) + K_deltai * self.error_omega_integral + K_deltad * error_omega_deriv
        # Saturation for steering
        if delta < -1:
            delta = -1
        if delta > 1:
            delta = 1
        
        print("v: {:5.2f} v_r: {:5.2f} e_v: {:5.2f}\t e_v_i: {:5.2f}\tthr: {:5.2f}\n"
              "Kp term:{:5.2f} Ki term: {:5.2f}".format(
            v_est, v, error_v, self.error_v_integral, throttle, K_th*error_v, K_thi*self.error_v_integral))
        print("omega: {:5.2f} omega_est: {:5.2f} e_del: {:5.2f}\t e_del_i: {:5.2f}\tdelta: {:5.2f}\n"
              "Kp term:{:5.2f} Ki term: {:5.2f}".format(
            omega, omega_est, error_omega, self.error_omega_integral, delta, K_delta*error_omega, K_deltai*self.error_omega_integral))

        # Set the history variables
        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta
        return v, omega, throttle, delta