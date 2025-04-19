#include "opennav_docking/nav_type_selector.hpp"

NavTypeSelector::NavTypeSelector(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {
    // Initialize node
    node_ = node;
    // Initialize publishers
    controller_selector_pub_ = node_->create_publisher<std_msgs::msg::String>("/controller_type", rclcpp::QoS(10).reliable().transient_local());
    goal_checker_selector_pub_ = node_->create_publisher<std_msgs::msg::String>("/goal_checker_type", rclcpp::QoS(10).reliable().transient_local());
    controller_function_pub_ = node_->create_publisher<std_msgs::msg::String>("/controller_function", rclcpp::QoS(10).reliable().transient_local());
    dock_controller_selector_pub_ = node_->create_publisher<std_msgs::msg::String>("/dock_controller_type", rclcpp::QoS(10).reliable().transient_local());
}

void NavTypeSelector::setType(std::string const & mode, char & offset_direction, geometry_msgs::msg::PoseStamped & original_staging_pose, double const & offset) {
    // Determine the goal checker type
    if(strstr(mode.c_str(), "precise") != nullptr) {
        goal_checker_selector_msg_.data = "Precise";
    } else if(strstr(mode.c_str(), "loose") != nullptr) {
        goal_checker_selector_msg_.data = "Loose";
    } else {
        goal_checker_selector_msg_.data = "Loose";
        RCLCPP_WARN(node_->get_logger(), "No goal checker type selected, defaulting to %s", goal_checker_selector_msg_.data.c_str());
    }
    goal_checker_selector_pub_->publish(goal_checker_selector_msg_);    // Publish the goal checker type

    // Determine the controller type
    if(strstr(mode.c_str(), "fast") != nullptr) {
        controller_selector_msg_.data = "Fast";
    } else if(strstr(mode.c_str(), "slow") != nullptr) {
        controller_selector_msg_.data = "Slow";
    } else if(strstr(mode.c_str(), "linearBoost") != nullptr) {
        controller_selector_msg_.data = "LinearBoost";
    } else if(strstr(mode.c_str(), "angularBoost") != nullptr) {
        controller_selector_msg_.data = "AngularBoost";
    } else {
        controller_selector_msg_.data = "Slow";
        RCLCPP_WARN(node_->get_logger(), "No controller type selected, defaulting to %s", controller_selector_msg_.data.c_str());
    }
    controller_selector_pub_->publish(controller_selector_msg_);    // Publish the controller type

    // Determine the controller function
    if(strstr(mode.c_str(), "delaySpin") != nullptr) {
        controller_function_msg_.data = "DelaySpin";
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m \"DelaySpin\" is activated \033[0m");
    } else {
        controller_function_msg_.data = "None";
    }
    controller_function_pub_->publish(controller_function_msg_);    // Publish the controller function

    // Determine the dock controller type
    if(strstr(mode.c_str(), "ordinary") != nullptr) {
        dock_controller_selector_msg_.data = "Ordinary";
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m Dock controller has set to \"Ordinary\" \033[0m");
    } else if(strstr(mode.c_str(), "gentle") != nullptr) {
        dock_controller_selector_msg_.data = "Gentle";
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m Dock controller has set to \"Gentle\" \033[0m");
    } else if(strstr(mode.c_str(), "rush") != nullptr) {
        dock_controller_selector_msg_.data = "Rush";
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m Dock controller has set to \"Rush\" \033[0m");
    } else {
        dock_controller_selector_msg_.data = "Ordinary";
        RCLCPP_WARN(node_->get_logger(), "No dock controller type selected, defaulting to %s", dock_controller_selector_msg_.data.c_str());
    }

    // Determine the offset direction & value
    if(strchr(mode.c_str(), 'x') != nullptr) {
        offset_direction = 'x';
        original_staging_pose.pose.position.x -= offset;
    } else if(strchr(mode.c_str(), 'y') != nullptr) {
        offset_direction = 'y';
        original_staging_pose.pose.position.y -= offset;
    } else {
        offset_direction = 'z';
        original_staging_pose.pose.position.x -= offset;
        original_staging_pose.pose.position.y -= offset;
        RCLCPP_WARN(node_->get_logger(), "No offset direction selected, applying offset to both x and y");
    }
}