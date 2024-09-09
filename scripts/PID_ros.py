#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Joy
from tf_transformations import euler_from_quaternion
import casadi as ca
import numpy as np
from corti.nvp_PIDcontrol import PIControl
from corti.rover_planning import RoverPlanner
from tf_transformations import euler_from_quaternion

v  = 1
r = 1
# planner = RoverPlanner(x=2.3, y=-0.8, v=v, theta=np.pi/2, r=r)
# planner.goto(2, 4.2, v, r)
# planner.goto(-2, 4.2, v, r)
# # planner.goto(2.23, 3, v, r)
# planner.stop(-2, 4.2)

x0 = 3.45
y0 = 2.62
theta0 = -3

xf = -5.24
yf = 3.6

planner = RoverPlanner(x=x0, y=y0, v=v, theta=theta0, r=r)
planner.goto(-5.24, 3.6, v, r)
planner.stop(xf, yf)

ref_data = planner.compute_ref_data(plot=False)
t = ref_data['t']
ref_x_list = ref_data['x'](t).tolist()
ref_y_list = ref_data['y'](t).tolist()
ref_theta_list = ref_data['theta'](t).tolist()


def find_next_wp(state, traj, curr_idx):
        max_idx = traj.shape[0]
        idx = curr_idx
        while idx<max_idx:
            if (ca.norm_2(state[:2] - traj[idx][:2]) < 1e-3):
                return idx
            else:
                idx+=1
        return curr_idx

class PIDPublisher(Node):
    def __init__(self):
        super().__init__('night_vapor_publisher')
        self.pub_control_input = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_joy = self.create_publisher(Joy, '/auto_joy', 10)
        self.sub_mocap = self.create_subscription(Odometry, '/nightvapor1/odom', self.pose_cb, 10)
        self.pub_ref_path = self.create_publisher(Path, '/ref_path', 10)
        self.pub_path = self.create_publisher(Path, '/path', 10)
        self.timer_path = self.create_timer(1, self.publish_ref_path)
        self.timer = self.create_timer(0.01, self.pub_night_vapor)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time = 0.5
        self.dt = 0.01
        self.pi_control = PIControl(self.dt)
        self.takeoff = False
        self.taxi = True
        self.takeoff_time = 0.0
        self.x_list = []
        self.y_list = []
        self.theta_list = []
        self.throttle = 1.0
        self.trail_size = 1000

    def pose_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # print(orientation_list)
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.theta = yaw
            
    def pub_night_vapor(self):
        # print(self.x, self.y, self.theta)
        x_ref = ref_data['x'](self.time)
        y_ref = ref_data['y'](self.time)

        # print(throttle, delta)
        if (np.sqrt((self.x - x_ref)**2 + (self.y - y_ref)**2) < 1) and (self.takeoff == False):
            self.time += self.dt
            self.taxi = True
            self.takeoff = False

        if (np.sqrt((self.x - (xf))**2 + (self.y - yf)**2) < 3):
            self.taxi = False
            self.takeoff = True

        if self.takeoff == True:
            # self.get_logger().log('takeoff')
            self.takeoff_time += self.dt
            self.throttle += -self.dt
            if self.throttle < -0.6:
                self.throttle = -0.6
            delta = 0
        if self.taxi == True:
            v, omega, self.throttle, delta = self.pi_control.compute_control(self.time, self.x, self.y, self.theta, ref_data)
        # self.get_logger().log(self.throttle)
        # print(self.time, ref_data['x'](self.time), ref_data['y'](self.time), ref_data['theta'](self.time), self.x, self.y, self.theta)

        # vel_msg = Twist()
        # vel_msg.linear.x = float(v)
        # vel_msg.angular.z = float(omega)
        # self.pub_control_input.publish(vel_msg)

        joy_msg = Joy()

        joy_msg.axes = [0.0]*5

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
        joy_msg.axes[2] = 0
        joy_msg.axes[3] = 0
        joy_msg.axes[4] = 1900
       
        self.pub_joy.publish(joy_msg)

        
        # Append current position to the list
        self.x_list.append(self.x)
        self.y_list.append(self.y)
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
            del self.theta_list[0]
        for x, y, theta in zip(self.x_list, self.y_list, self.theta_list):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'qualisys'
            pose.pose.position.x = x
            pose.pose.position.y = y
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
