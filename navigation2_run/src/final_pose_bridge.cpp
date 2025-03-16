// This is a bridge node which subscribes to the /final_pose topic from localization with data type /geometry_msgs/msg/pose_with_covariance_stamped &
// the velocity feedback topic from chassis with data type /geometry_msgs/msg/twist,
// then publishes the combined data as /final_pose_nav topic with data type /nav_msgs/msg/odometry

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PoseToOdometryBridge : public rclcpp::Node
{
public:
    PoseToOdometryBridge()
        : Node("final_pose_bridge")
    {
        // Declare and get the parameter for the Twist topic name from the YAML file
        this->declare_parameter<std::string>("twist_topic", "/driving_duiduidui");
        std::string twist_topic_ = this->get_parameter("twist_topic").as_string();

        // Subscribe to /final_pose topic (PoseWithCovarianceStamped)
        subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/final_pose", 10, std::bind(&PoseToOdometryBridge::pose_callback, this, std::placeholders::_1));

        // Subscribe to the twist topic (Twist) using the parameter-loaded topic name
        subscription_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
            twist_topic_, 10, std::bind(&PoseToOdometryBridge::twist_callback, this, std::placeholders::_1));

        // Publisher to /final_pose_nav topic (Odometry)
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/final_pose_nav", 10);

        // Timer for publishing the Odometry message
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&PoseToOdometryBridge::timer_callback, this));
    }

private:
    // PoseWithCovarianceStamped message callback
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Set the header from PoseWithCovarianceStamped
        odom_msg_.header = msg->header;

        // Extract Pose (position and orientation) and covariance, and set it in Odometry
        odom_msg_.pose.pose = msg->pose.pose;
        odom_msg_.pose.covariance = msg->pose.covariance;

        // RCLCPP_INFO(this->get_logger(), "Published Odometry with Pose to /final_pose_nav");
    }

    // Twist message callback
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Set the Twist part of the Odometry message (velocity)
        odom_msg_.twist.twist = *msg;

        // RCLCPP_INFO(this->get_logger(), "Published Odometry with Twist to /final_pose_nav");
    }

    // Timer callback for publishing the Odometry message
    void timer_callback()
    {
        // Publish the Odometry message
        publisher_->publish(odom_msg_);
    }

    // Subscription for PoseWithCovarianceStamped messages
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose_;

    // Subscription for Twist messages (loaded from parameter)
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_twist_;

    // Publisher for Odometry messages
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    nav_msgs::msg::Odometry odom_msg_;

    // Set timer for publishing the Odometry message
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Start spinning the node to process subscriptions and callbacks
    rclcpp::spin(std::make_shared<PoseToOdometryBridge>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}