#include "nav2_behavior_tree/plugins/decorator/stop_controller.hpp"

namespace nav2_behavior_tree
{
  StopController::StopController(
    const std::string & name,
    const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf),
      stop_robot(false),
      shrink_completed(false),
      do_shrinkback(false)
  {
    // Retrieve node from the blackboard
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // Create publisher for cmd_vel
    cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create a callback group and add it to our executor
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Create client to shrink's service
    shrink_client = node_->create_client<std_srvs::srv::SetBool>(
      "/shrink/doneShrink", 
      rmw_qos_profile_services_default);

    // Create subscriber for the stop topic with subscription options
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    stop_sub = node_->create_subscription<std_msgs::msg::Bool>(
      "/stopRobot", 
      rclcpp::SystemDefaultsQoS(),
      std::bind(&StopController::stopCallback, this, std::placeholders::_1),
      sub_options);
    
    shrinkback_sub = node_->create_subscription<std_msgs::msg::Bool>(
      "/shrink/shrinkback", 
      rclcpp::SystemDefaultsQoS(),
      std::bind(&StopController::shrinkBackCallBack, this, std::placeholders::_1), // Match case with actual method name
      sub_options);

    goal_reach_sub = node_->create_subscription<std_msgs::msg::Bool>(
      "/goal_reached", 
      rclcpp::SystemDefaultsQoS(),
      std::bind(&StopController::goalReachCallBack, this, std::placeholders::_1), // Match case with actual method name
      sub_options);

  }

  void StopController::stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    stop_robot = msg->data;
  }

  void StopController::shrinkBackCallBack(const std_msgs::msg::Bool::SharedPtr msg)
  {
    shrink_completed = msg->data;
    RCLCPP_INFO(node_->get_logger(), "Received shrinkback message: %s", msg->data ? "true" : "false");
  }

  void StopController::goalReachCallBack(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Received goal_reach message: %s", msg->data ? "true" : "false");
    if(!shrink_completed) do_shrinkback = false;
    else do_shrinkback = msg->data;
  }

  void StopController::checkIfShrinkRequest()
  {
    if (!shrink_client->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_ERROR(node_->get_logger(), "Shrink service not available, waiting...");
    } else {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;  // Instruct shrink to call setToOriginal()
      auto result = shrink_client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        if (response->success)
          RCLCPP_INFO(node_->get_logger(), "Shrink request sent successfully");
        else
          RCLCPP_ERROR(node_->get_logger(), "Shrink service call failed");
      }
      else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send shrink service call");
      }
    }
  }

  inline BT::NodeStatus StopController::tick()
  {
    // Optionally, spin the executor here too.
    callback_group_executor_.spin_some();

    // If we received a shrink completed message, increment its timer.
    if (shrink_completed) {
      if(do_shrinkback){
        checkIfShrinkRequest();
        do_shrinkback = false;
        shrink_completed = false;
      }
    }



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