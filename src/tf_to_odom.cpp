#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <nav_msgs/msg/detail/odometry__struct.hpp>

using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node {
public:
    OdomPublisher()
        : Node("tf_to_odom")
    {
        m_target_frame = this->declare_parameter<std::string>("target_frame", "base_link");
        m_base_frame = this->declare_parameter<std::string>("base_frame", "map");

        // tranform buffer
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        m_pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Call on_timer function every second
        m_timer = this->create_wall_timer(
            1s, std::bind(&OdomPublisher::on_timer, this));
    }

private:
    void on_timer()
    {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = m_tf_buffer->lookupTransform(
                m_base_frame, m_target_frame,
                tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                m_base_frame.c_str(), m_target_frame.c_str(), ex.what());
            return;
        }
        nav_msgs::msg::Odometry msg;
        msg.child_frame_id = m_target_frame;
        msg.header.stamp = tf.header.stamp;
        msg.header.frame_id = m_base_frame;
        msg.pose.pose.position.x = tf.transform.translation.x;
        msg.pose.pose.position.y = tf.transform.translation.y;
        msg.pose.pose.position.z = tf.transform.translation.z;
        msg.pose.pose.orientation.x = tf.transform.rotation.x;
        msg.pose.pose.orientation.y = tf.transform.rotation.y;
        msg.pose.pose.orientation.z = tf.transform.rotation.z;
        msg.pose.pose.orientation.w = tf.transform.rotation.w;
        m_pub_odom->publish(msg);
    }

    std::string m_target_frame;
    std::string m_base_frame;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener { nullptr };
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer { nullptr };
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_odom { nullptr };
    rclcpp::TimerBase::SharedPtr m_timer { nullptr };
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
