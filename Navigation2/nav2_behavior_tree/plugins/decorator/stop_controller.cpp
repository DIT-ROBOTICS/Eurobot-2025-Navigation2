#include "nav2_behavior_tree/plugins/decorator/stop_controller.hpp"

namespace nav2_behavior_tree
{
  StopController::StopController(
    const std::string & name,
    const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf),
      stop_robot(false)
  {
    // Retrieve node from the blackboard
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // Create a callback group and add it to our executor
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Create subscriber for the stop topic with subscription options
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    stop_sub = node_->create_subscription<std_msgs::msg::Bool>(
      "/stopRobot", 
      rclcpp::SystemDefaultsQoS(),
      std::bind(&StopController::stopCallback, this, std::placeholders::_1),
      sub_options);
  }

  void StopController::stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    stop_robot = msg->data;
  }

  inline BT::NodeStatus StopController::tick()
  {
    callback_group_executor_.spin_some();

    if (stop_robot) {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      cmd_vel_pub->publish(cmd_vel);
      RCLCPP_INFO(node_->get_logger(), "\033[1;31m running in stop_controller \033[0m");
      return BT::NodeStatus::FAILURE;
    }
    return child_node_->executeTick();
  }
}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::StopController>("StopController");
}