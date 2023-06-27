#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/rate.hpp>

class FramePublisher : public rclcpp::Node {
public:
    FramePublisher()
        : Node("odom_to_tf")
    {
        m_async = this->declare_parameter<bool>("async", false);
        m_sync_dt = this->declare_parameter<double>("sync_dt", 0.02);

        m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        m_sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&FramePublisher::handle_odom, this, std::placeholders::_1));

        if (not m_async) {
            m_timer = this->create_wall_timer(
                std::chrono::duration<float> { m_sync_dt },
                std::bind(&FramePublisher::on_timer, this));
        }
    }

private:
    void handle_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
        if (m_async) {
            publish_tf(msg);
        } else {
            m_odom = msg;
        }
    }

    void on_timer()
    {
        if (m_odom != NULL) {
            publish_tf(m_odom);
        }
    }

    void publish_tf(const std::shared_ptr<nav_msgs::msg::Odometry>)
    {
        geometry_msgs::msg::TransformStamped t;

        // header
        if (m_async) {
            t.header.stamp = m_odom->header.stamp;
        } else {
            t.header.stamp = this->get_clock()->now();
        }
        t.header.frame_id = m_odom->header.frame_id;
        t.child_frame_id = m_odom->child_frame_id;

        // position
        t.transform.translation.x = m_odom->pose.pose.position.x;
        t.transform.translation.y = m_odom->pose.pose.position.y;
        t.transform.translation.z = m_odom->pose.pose.position.z;

        // attitude
        t.transform.rotation.x = m_odom->pose.pose.orientation.x;
        t.transform.rotation.y = m_odom->pose.pose.orientation.y;
        t.transform.rotation.z = m_odom->pose.pose.orientation.z;
        t.transform.rotation.w = m_odom->pose.pose.orientation.w;

        // Send the transformation
        m_tf_broadcaster->sendTransform(t);
    }

    // attributes
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_odom;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<nav_msgs::msg::Odometry> m_odom { NULL };
    bool m_async;
    float m_sync_dt;
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
