#include "nav2_behavior_tree/plugins/decorator/stop_controller.hpp"

namespace nav2_behavior_tree
{
    StopController::StopController(
        const std::string & name,
        const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf),
      stop_robot(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        stop_sub = node_->create_subscription<std_msgs::msg::Bool>(
            "/stopRobot",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&StopController::stopCallback, this, std::placeholders::_1),
            sub_option);
    }

    inline BT::NodeStatus StopController::tick()
    {
        callback_group_executor_.spin_some();
        if (stop_robot)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub->publish(cmd_vel);
            RCLCPP_INFO(node_->get_logger(), "running in stop_controller");
            return BT::NodeStatus::FAILURE;
        }
        return child_node_->executeTick();
    }

    void StopController::stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        stop_robot = msg->data;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::StopController>("StopController");
}