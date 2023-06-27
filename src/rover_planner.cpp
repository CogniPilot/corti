#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "synapse_msgs/msg/bezier_curve.hpp"
#include "synapse_msgs/msg/bezier_trajectory.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "casadi/bezier6.h"

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RoverPlanner : public rclcpp::Node {
public:
    RoverPlanner()
        : Node("rover_planner")
    {
        this->declare_parameter("avg_vel", 1.0);
        this->declare_parameter("vel0", 1.0);
        this->declare_parameter("vel1", 0.1);

        // tranform buffer
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

        // publications
        m_pub_traj = this->create_publisher<synapse_msgs::msg::BezierTrajectory>("traj", 10);
        m_pub_path = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        // subscriptions
        m_sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&RoverPlanner::goal_callback, this, _1));
    }

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::string childFrame = "base_link";
        std::string baseFrame = "map";
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = m_tf_buffer->lookupTransform(
                baseFrame, childFrame,
                tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                baseFrame.c_str(), childFrame.c_str(), ex.what());
            return;
        }
        std::cout << "X:" << tf.transform.translation.x << std::endl;
        std::cout << "Y:" << tf.transform.translation.y << std::endl;

        double delta_x = msg->pose.position.x - tf.transform.translation.x;
        double delta_y = msg->pose.position.y - tf.transform.translation.y;
        double dist = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        double avg_vel = this->get_parameter("avg_vel").get_parameter_value().get<double>();
        double vel0 = this->get_parameter("vel0").get_parameter_value().get<double>();
        double vel1 = this->get_parameter("vel1").get_parameter_value().get<double>();
        casadi_real T = dist / avg_vel;

        // solve for PX, PY
        // bezier6_solve:(wp_0[2],wp_1[2],T)->(P[1x6])
        casadi_real PX[6] = {};
        casadi_real PY[6] = {};
        {
            double psi0 = 2 * atan2(tf.transform.rotation.z, tf.transform.rotation.w);
            double psi1 = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);

            const casadi_real* arg[3] = {};
            casadi_real* res[1] = {};
            casadi_real wpx0[2] = {
                tf.transform.translation.x,
                vel0 * cos(psi0)
            };
            casadi_real wpy0[2] = {
                tf.transform.translation.y,
                vel0 * sin(psi0)
            };
            casadi_real wpx1[2] = {
                msg->pose.position.x, vel1 * cos(psi1)
            };
            casadi_real wpy1[2] = {
                msg->pose.position.y, vel1 * sin(psi1)
            };

            // solve for PX
            arg[0] = wpx0;
            arg[1] = wpx1;
            arg[2] = &T;
            res[0] = PX;
            casadi_int* iw = NULL;
            casadi_real* w = NULL;
            int mem = 0;
            bezier6_solve(arg, res, iw, w, mem);

            // solve for PY
            arg[0] = wpy0;
            arg[1] = wpy1;
            res[0] = PY;
            bezier6_solve(arg, res, iw, w, mem);

            synapse_msgs::msg::BezierTrajectory msg;
            msg.header.frame_id = "map";

            msg.time_start = get_clock()->now().nanoseconds();
            synapse_msgs::msg::BezierCurve curve;
            curve.time_stop = T * 1e9 + msg.time_start;
            for (int i = 0; i < 6; i++) {
                curve.x.push_back(PX[i]);
                curve.y.push_back(PY[i]);
                curve.z.push_back(0);
            }
            msg.curves.push_back(curve);
            m_pub_traj->publish(msg);
        }

        // visualize in rviz
        {
            // bezier6_traj:(t,T,P[1x6])->(r[3])
            const casadi_real* arg[3] = {};
            casadi_real* res[1] = {};
            casadi_int* iw = NULL;
            casadi_real* w = NULL;
            casadi_real rx[3], ry[3] = {};
            int mem = 0;
            int traj_steps = 10;

            auto path = nav_msgs::msg::Path();
            path.header.frame_id = "map";
            auto pose = geometry_msgs::msg::PoseStamped();
            pose.header.frame_id = "map";
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            for (int i = 0; i < traj_steps; i++) {
                double ti = i * T / traj_steps;
                arg[0] = &ti;
                arg[1] = &T;
                arg[2] = PX;
                res[0] = rx;
                bezier6_traj(arg, res, iw, w, mem);
                arg[2] = PY;
                res[0] = ry;
                bezier6_traj(arg, res, iw, w, mem);
                double psi = atan2(ry[1], rx[1]);
                pose.pose.position.x = rx[0];
                pose.pose.position.y = ry[0];
                pose.pose.orientation.w = cos(psi / 2);
                pose.pose.orientation.z = sin(psi / 2);
                path.poses.push_back(pose);
            }
            m_pub_path->publish(path);
        }
    }

    rclcpp::Publisher<synapse_msgs::msg::BezierTrajectory>::SharedPtr m_pub_traj;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pub_path;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_sub_goal;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener { nullptr };
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPlanner>());
    rclcpp::shutdown();
    return 0;
}
