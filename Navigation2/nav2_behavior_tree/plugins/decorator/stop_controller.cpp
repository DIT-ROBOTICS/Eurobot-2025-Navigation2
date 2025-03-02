#include "nav2_behavior_tree/plugins/decorator/stop_controller.hpp"

namespace nav2_behavior_tree
{
    StopController::StopController(
        const std::string & name,
        const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf),
      stop_robot(true)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        RCLCPP_INFO(node_->get_logger(), "Creating StopController BT node");
        cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        stop_sub = node_->create_subscription<std_msgs::msg::Bool>("/stopRobot", 10, std::bind(&StopController::stopCallback, this, std::placeholders::_1));
    }

    BT::NodeStatus StopController::tick()
    {
        if (stop_robot)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub->publish(cmd_vel);
            stop_robot = true;
            RCLCPP_INFO(node_->get_logger(), "Robot stopped");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "StopController ticking");
        return child_node_->executeTick();
    }

    void StopController::stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "In call back");
        stop_robot = msg->data;
        RCLCPP_INFO(node_->get_logger(), "Stop signal received");
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::StopController>("StopController");
}