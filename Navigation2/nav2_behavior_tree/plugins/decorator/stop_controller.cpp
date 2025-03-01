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
        cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        stop_sub = node_->create_subscription<std_msgs::msg::Bool>("stop", 10, std::bind(&StopController::stopCallback, this, std::placeholders::_1));
    }

    BT::NodeStatus StopController::tick()
    {
        if (stop_robot)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub->publish(cmd_vel);
            stop_robot = false;
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