#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav2_util/node_utils.hpp"


namespace nav2_behavior_tree
{
  class StopController : public BT::DecoratorNode
  {
  public:
    StopController(const std::string & name, const BT::NodeConfiguration & conf);
    static BT::PortsList providedPorts(){
      return { BT::InputPort<bool>("stop") };
    }
    BT::NodeStatus tick() override;

  private:
    bool stop_robot;
    bool shrink_completed;
    bool do_shrinkback;
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shrinkback_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reach_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr shrink_client;


    void stopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void checkIfShrinkRequest();
    void shrinkBackCallBack(const std_msgs::msg::Bool::SharedPtr msg);
    void goalReachCallBack(const std_msgs::msg::Bool::SharedPtr msg);
  };
}

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_