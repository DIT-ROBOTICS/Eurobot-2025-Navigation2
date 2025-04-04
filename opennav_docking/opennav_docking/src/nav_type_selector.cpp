#include "opennav_docking/nav_type_selector.hpp"

NavTypeSelector::NavTypeSelector(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {
    // Initialize node
    node_ = node;
    // Initialize publishers
    controller_selector_pub_ = node_->create_publisher<std_msgs::msg::String>("/controller_type", rclcpp::QoS(10).reliable().transient_local());
    goal_checker_selector_pub_ = node_->create_publisher<std_msgs::msg::String>("/goal_checker_type", rclcpp::QoS(10).reliable().transient_local());
}

void NavTypeSelector::setType(std::string const & mode, char & offset_direction, geometry_msgs::msg::PoseStamped & original_staging_pose, double const & offset) {
    // Determine the goal checker type
    if(strstr(mode.c_str(), "percise") != NULL) {
        goal_checker_selector_msg_.data = "Percise";
    } else if(strstr(mode.c_str(), "loose") != NULL) {
        goal_checker_selector_msg_.data = "Loose";
    } else {
        goal_checker_selector_msg_.data = "percise";
        RCLCPP_WARN(node_->get_logger(), "No goal checker type selected, defaulting to percise");
    }
    goal_checker_selector_pub_->publish(goal_checker_selector_msg_);    // Publish the goal checker type

    // Determine the controller type
    if(strstr(mode.c_str(), "fast") != NULL) {
        controller_selector_msg_.data = "Fast";
    } else if(strstr(mode.c_str(), "slow") != NULL) {
        controller_selector_msg_.data = "Slow";
    } else {
        controller_selector_msg_.data = "slow";
        RCLCPP_WARN(node_->get_logger(), "No controller type selected, defaulting to fast");
    }
    controller_selector_pub_->publish(controller_selector_msg_);    // Publish the controller type

    // Determine the offset direction & value
    if(strchr(mode.c_str(), 'x') != NULL) {
        offset_direction = 'x';
        original_staging_pose.pose.position.x -= offset;
    } else if(strchr(mode.c_str(), 'y') != NULL) {
        offset_direction = 'y';
        original_staging_pose.pose.position.y -= offset;
    } else {
        offset_direction = 'z';
        original_staging_pose.pose.position.x -= offset;
        original_staging_pose.pose.position.y -= offset;
        RCLCPP_WARN(node_->get_logger(), "No offset direction selected, applying offset to both x and y");
    }
    
}