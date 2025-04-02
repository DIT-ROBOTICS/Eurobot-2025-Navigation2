#include "opennav_docking/nav_type_selector.hpp"

NavTypeSelector::NavTypeSelector(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {
    // Initialize node
    node_ = node;
    // Initialize publishers
    controller_selector_pub_ = node_->create_publisher<std_msgs::msg::String>("/controller_type", rclcpp::QoS(10).reliable().transient_local());
    goal_checker_selector_pub_ = node_->create_publisher<std_msgs::msg::String>("/goal_checker_type", rclcpp::QoS(10).reliable().transient_local());
}

void NavTypeSelector::setType(std::string const & mode, char & offset_direction, geometry_msgs::msg::PoseStamped & original_staging_pose, double const & offset) {
    if (mode == "dock_x_percise_fast") {
        original_staging_pose.pose.position.x -= offset;
        offset_direction = 'x';
        controller_selector_msg_.data = "Fast";
        goal_checker_selector_msg_.data = "Percise";
    } else if (mode == "dock_y_percise_fast") {
        original_staging_pose.pose.position.y -= offset;
        offset_direction = 'y';
        controller_selector_msg_.data = "Fast";
        goal_checker_selector_msg_.data = "Percise";
    } else if (mode == "dock_x_percise_slow") {
        original_staging_pose.pose.position.x -= offset;
        offset_direction = 'x';
        controller_selector_msg_.data = "Slow";
        goal_checker_selector_msg_.data = "Percise";
    } else if (mode == "dock_y_percise_slow") {
        original_staging_pose.pose.position.y -= offset;
        offset_direction = 'y';
        controller_selector_msg_.data = "Slow";
        goal_checker_selector_msg_.data = "Percise";
    } else if (mode == "dock_x_loose_fast") {
        original_staging_pose.pose.position.x -= offset;
        offset_direction = 'x';
        controller_selector_msg_.data = "Fast";
        goal_checker_selector_msg_.data = "Loose";
    } else if (mode == "dock_y_loose_fast") {
        original_staging_pose.pose.position.y -= offset;
        offset_direction = 'y';
        controller_selector_msg_.data = "Fast";
        goal_checker_selector_msg_.data = "Loose";
    } else {
        RCLCPP_WARN(node_->get_logger(), "Unknown dock type: %s. Use navigate Slow & Percise for default", mode.c_str());
        controller_selector_msg_.data = "Slow";
        goal_checker_selector_msg_.data = "Percise";
        return;
    }
    
    // Publish the selected controller and goal checker types
    // ! goal checker must be published first
    // TODO: check if the data received successfully
    goal_checker_selector_pub_->publish(goal_checker_selector_msg_);
    controller_selector_pub_->publish(controller_selector_msg_);
}