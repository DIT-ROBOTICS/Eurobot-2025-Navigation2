#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <thread>


namespace nav2_behavior_tree
{
  class StopController : public BT::DecoratorNode
  {
  public:
    StopController(const std::string & name, const BT::NodeConfiguration & conf);
    ~StopController();
    static BT::PortsList providedPorts(){
      return { BT::InputPort<bool>("stop") };
    }
    BT::NodeStatus tick() override;

  private:
    bool stop_robot;
    bool shrink_completed;
    int shrink_completed_timer;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr shrink_completed_srv;
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr shrink_client;

    // Dedicated thread for spinning the callback group
    std::thread spin_thread_;

    void stopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void checkIfShrinkRequest();
    void shrinkCompletedCallback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  };
}

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_CONTROLLER_HPP_