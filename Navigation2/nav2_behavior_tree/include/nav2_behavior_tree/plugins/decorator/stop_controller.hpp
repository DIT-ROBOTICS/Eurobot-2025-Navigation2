#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_behavior_tree
{
    class StopController : public BT::DecoratorNode
    {
        public:
            StopController(
                const std::string & name,
                const BT::NodeConfiguration & conf);
            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<bool>("stop")
                };
            }
            BT::NodeStatus tick() override;
        private:
            void stopCallback(const std_msgs::msg::Bool::SharedPtr msg);
            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
            bool stop_robot;
    };
}

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_