#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from tf_transformations import euler_from_quaternion
import casadi as ca
import numpy as np
from corti.nvp_PIDcontrol import PIDControl
from corti.rover_planning import RoverPlanner

from corti.dubin_planning import gen_waypoints, gen_reference_trajectory

r = 1.22 # turning radius
n = 100 # number of turning points

x0 = 1.14
y0 = 8.92
theta0 = -1.2

xf = -3.81
yf = 3.26

control_point = [(x0, y0), (5.6, 1), (2.22, 2.34), (xf, yf)]
wp = gen_waypoints(control_point, r, n)
ref = gen_reference_trajectory(wp, 0.5, 0.01) # wp, vel, dt

ref_x_list = ref[:,0].tolist()
ref_y_list = ref[:,1].tolist()
ref_theta_list = ref[:,2].tolist()

ref_v_list = []
ref_omega_list = []
for i in range(ref.shape[0]):
    if i == 0:
        v = 0
        omega = 0
    else:
        v = np.abs(np.sqrt(ref[i,0]**2+ref[i,1]**2) - np.sqrt(ref[i-1,0]**2+ref[i-1,1]**2))/0.01
        omega = (ref[i,2] - ref[i-1,2])/0.01
    ref_v_list.append(v)
    ref_omega_list.append(omega)

ref_data = {
        'x': ref_x_list,
        'y': ref_y_list,
        'theta': ref_theta_list,
        'omega': ref_omega_list,
        'V': ref_v_list,
    }


# v  = 0.8
# r = 0.5

# # nvp2 waypoints
# x0 = 1.14
# y0 = 8.92
# theta0 = -1.2

# xf = -3.81
# yf = 3.26
# planner = RoverPlanner(x=x0, y=y0, v=v, theta=theta0, r=r)
# planner.goto(5.6, 1, v, 2)
# # planner.goto(2.54, 2, v, 1.5)
# planner.goto(xf, yf, v, 0.5)
# planner.stop(xf, yf)

# ref_data = planner.compute_ref_data(plot=False)
# t = ref_data['t']
# ref_x_list = ref_data['x'](t).tolist()
# ref_y_list = ref_data['y'](t).tolist()
# ref_theta_list = ref_data['theta'](t).tolist()


class PIDPublisher(Node):
    def __init__(self):
        super().__init__('night_vapor_publisher')
        # self.pub_control_input = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_joy = self.create_publisher(Joy, '/nv1/auto_joy', 10)
        self.sub_mocap = self.create_subscription(Odometry, '/nightvapor1/odom', self.pose_cb, 10)
        self.pub_ref_point = self.create_publisher(PoseStamped, '/ref_point1', 10)
        self.pub_ref_path = self.create_publisher(Path, '/ref_path', 10)
        self.pub_path = self.create_publisher(Path, '/path1', 10)
        self.timer_path = self.create_timer(1, self.publish_ref_path)
        self.timer = self.create_timer(0.01, self.pub_night_vapor)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.time = 0
        self.dt = 0.01
        self.pi_control = PIDControl(self.dt, 1)
        self.takeoff = False
        self.taxi = True
        self.takeoff_time = 0.0
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.theta_list = []
        self.throttle = 1.0
        self.trail_size = 1000
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.omega_est = None
        self.omega_est_last = None
        self.v_est_last = None
        self.v_est = None


    def pose_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.theta = yaw

        if self.prev_x is None:
            self.prev_x = self.x
            self.prev_y = self.y
            self.prev_theta = self.theta
        alpha = np.exp(-2*np.pi*10*self.dt) # exp(w*T)
        # alpha = 0.001
        v_est_new = np.abs((np.sqrt(self.x**2+self.y**2) - np.sqrt(self.prev_x**2+self.prev_y**2))/self.dt)
        if self.v_est_last is None:
            self.v_est_last = v_est_new
        self.v_est = v_est_new*alpha + self.v_est_last*(1-alpha)
        #print('v: {:10.2f}'.format(self.v_est))
        
        omega_est_new = (self.theta - self.prev_theta)
        if self.omega_est_last is None:
            self.omega_est_last = omega_est_new
        self.omega_est = omega_est_new*alpha + self.omega_est_last*(1 - alpha)
            
        # print(orientation_list)
        # print(self.v_est)
        self.omega_est_last = self.omega_est
        self.v_est_last = self.v_est
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta

    def pub_night_vapor(self):
        if (self.pi_control.chi.x > -0.5) and (self.takeoff == False):
            self.time += self.dt
            self.taxi = True
            self.takeoff = False

        if (np.sqrt((self.x - (xf))**2 + (self.y - yf)**2) < 3):
            self.taxi = False
            self.takeoff = True

        if self.takeoff == True:
            print('takeoff')
            self.takeoff_time += self.dt
            self.throttle += 2*self.dt
            if self.throttle < 0.7:
                self.throttle = 0.7
            delta = 0
            elevator = -0.2

        if self.taxi == True:
            # print('taxi')
            v, omega, self.throttle, delta = self.pi_control.compute_control(int(self.time/self.dt), self.x, self.y, self.theta, ref_data, self.v_est, self.omega_est)
            elevator = 0
            # print(self.throttle, delta)
        # print(self.time, ref_data['x'](self.time), ref_data['y'](self.time), ref_data['theta'](self.time), self.x, self.y, self.theta)

        # vel_msg = Twist()
        # vel_msg.linear.x = float(v)
        # vel_msg.angular.z = float(omega)
        # self.pub_control_input.publish(vel_msg)

        ref_pos_msg = PoseStamped()
        ref_pos_msg.header.frame_id = 'qualisys'
        ref_pos_msg.pose.position.x = float(ref_data['x'][int(self.time/self.dt)])
        ref_pos_msg.pose.position.y = float(ref_data['y'][int(self.time/self.dt)])
        theta = float(ref_data['theta'][int(self.time/self.dt)])
        ref_pos_msg.pose.orientation.w = np.cos(theta/2)
        ref_pos_msg.pose.orientation.z = np.sin(theta/2)
        self.pub_ref_point.publish(ref_pos_msg)


        joy_msg = Joy()

        joy_msg.axes = [0.0]*5
        # print(self.throttle)
        joy_msg.axes[0] = self.throttle
        joy_msg.axes[1] = delta
        # if self.time <= 3.5:
        #     joy_msg.axes[0] = 0.5 #1200 # throttle
        #     joy_msg.axes[1] = 0 #1500 # rudder
        # elif (3.5 < self.time <= 5.5):
        #     joy_msg.axes[0] = 0.6
        #     joy_msg.axes[1] = -0.5
        # elif (5.5< self.time <= 8.5):
        #     joy_msg.axes[0] = 0.5 #1200 # throttle
        #     joy_msg.axes[1] = 0 #1500 # rudder
        # else:
        #     joy_msg.axes[0] = 1
        joy_msg.axes[2] = elevator
        joy_msg.axes[3] = 0
        joy_msg.axes[4] = 1900
       
        self.pub_joy.publish(joy_msg)

        
        # Append current position to the list
        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.z_list.append(self.z)
        self.theta_list.append(self.theta)
        #Publish the trajectory of the vehicle
        self.publish_path()

    def publish_ref_path(self):
        msg_path = Path()
        msg_path.header.frame_id = 'qualisys'
        msg_path.header.stamp = self.get_clock().now().to_msg()
        for x, y, theta in zip(ref_x_list, ref_y_list, ref_theta_list):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'qualisys'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = np.cos(theta/2)
            pose.pose.orientation.z = np.sin(theta/2)
            msg_path.poses.append(pose)
        self.pub_ref_path.publish(msg_path)

    def publish_path(self):
        msg_path = Path()
        msg_path.header.frame_id = 'qualisys'
        msg_path.header.stamp = self.get_clock().now().to_msg()
        if len(self.x_list) > self.trail_size:
            del self.x_list[0]
            del self.y_list[0]
            del self.z_list[0]
            del self.theta_list[0]
        for x, y, z, theta in zip(self.x_list, self.y_list, self.z_list, self.theta_list):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'qualisys'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = np.cos(theta/2)
            pose.pose.orientation.z = np.sin(theta/2)
            msg_path.poses.append(pose)
        self.pub_path.publish(msg_path)      
    

def main(args=None):
    rclpy.init(args=args)
    PID_publisher = PIDPublisher()
    rclpy.spin(PID_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    PID_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
