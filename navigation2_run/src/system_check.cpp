#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"

using namespace std::chrono_literals;

class SystemCheck : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using DockRobot = opennav_docking_msgs::action::DockRobot;

  SystemCheck()
  : Node("SystemCheck"), is_main_ready_(false)
  {
    ready_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/startup/plan", 10,
      std::bind(&SystemCheck::readyCallback, this, std::placeholders::_1));

    ready_srv_client_ = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>(
      "/robot/startup/ready_signal");

    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    dock_robot_client_ = rclcpp_action::create_client<DockRobot>(this, "dock_robot");

    RCLCPP_INFO(this->get_logger(), "\033[1;35m SystemCheck started, waiting for startup plan... \033[0m");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ready_sub_;
  rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<DockRobot>::SharedPtr dock_robot_client_;
  bool is_main_ready_;

  void readyCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg == nullptr || is_main_ready_)
      return;

    // RCLCPP_INFO(this->get_logger(), "\033[1;35m Received startup plan message, processing... \033[0m");

    // Retry until service is ready
    while (!ready_srv_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for ReadySignal service...");
      if (!rclcpp::ok()) return;
    }

    // Retry until navigate_to_pose is available
    while (!navigate_to_pose_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for navigate_to_pose action server...");
      if (!rclcpp::ok()) return;
    }

    // Retry until dock_robot is available
    while (!dock_robot_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for dock_robot action server...");
      if (!rclcpp::ok()) return;
    }

    // All systems ready
    is_main_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "\033[1;32m All systems ready !!! \033[0m");
    sendReadySignal(3, 3);  // group = 3 (navigation), state = 3 (START)
  }

  void sendReadySignal(int group, int state)
  {
    auto request = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();
    request->group = group;
    request->state = state;

    // RCLCPP_INFO(this->get_logger(), "\033[1;35m Sending ReadySignal (group=%d, state=%d)... \033[0m", group, state);

    ready_srv_client_->async_send_request(request,
      [this](rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m ReadySignal SUCCESS: group=%d \033[0m", response->group);
            is_main_ready_ = false; // Reset for next system check process
        } else {
            RCLCPP_WARN(this->get_logger(), "ReadySignal FAILED");
        }
      });
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemCheck>();
  rclcpp::spin(node);
  return 0;
}
