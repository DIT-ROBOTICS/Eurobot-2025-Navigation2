#include "nav2_behavior_tree/plugins/decorator/stop_controller.hpp"

namespace nav2_behavior_tree
{
  StopController::StopController(
    const std::string & name,
    const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf),
      stop_robot(false),
      shrink_completed(false),
      shrink_completed_timer(0)
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

    // Create service for receiving shrink completed notification
    // Try this version without the callback_group parameter
    shrink_completed_srv = node_->create_service<std_srvs::srv::SetBool>(
      "/stop_controller/shrink_completed",
      std::bind(&StopController::shrinkCompletedCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Start a dedicated spin thread for the callback group executor
    spin_thread_ = std::thread([this]() {
      rclcpp::Rate rate(50);  // 50Hz spinning rate
      while (rclcpp::ok()) {
        callback_group_executor_.spin_some();
        rate.sleep();
      }
    });
  }

  StopController::~StopController()
  {
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void StopController::shrinkCompletedCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    (void)request;  // Mark parameter as unused
    RCLCPP_INFO(node_->get_logger(), "Received shrink completed notification from shrink");
    // Set the flag and reset associated timer
    shrink_completed = true;
    shrink_completed_timer = 0;
    response->success = true;
    response->message = "Shrink completed flag set";
  }

  void StopController::stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    stop_robot = msg->data;
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
      shrink_completed_timer++;
      // If timer expires (e.g., 100 ticks ~ 5 seconds at 20Hz).
      if (shrink_completed_timer > 100) {
        RCLCPP_INFO(node_->get_logger(), "Timer expired, requesting shrink service to set original");
        checkIfShrinkRequest();
        shrink_completed = false;
        shrink_completed_timer = 0;
      }
    }

    if (stop_robot) {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      cmd_vel_pub->publish(cmd_vel);
      RCLCPP_INFO(node_->get_logger(), "running in stop_controller");
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