#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class SystemCheck : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using DockRobot = opennav_docking_msgs::action::DockRobot;

  SystemCheck()
  : Node("system_check"), running_(false), obstacle_check_count_(10)
  {
    ready_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/startup/plan", 10,
      std::bind(&SystemCheck::readyCallback, this, std::placeholders::_1));

    costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::QoS(10),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          latest_costmap_ = msg;
      });

    subscription_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/final_pose_nav", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_pose_ = msg;
      });

    ready_srv_client_ = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>(
      "/robot/startup/ready_signal");

    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    dock_robot_client_ = rclcpp_action::create_client<DockRobot>(this, "dock_robot");

    this->declare_parameter("costmap_tolerance", 70);
    this->get_parameter("costmap_tolerance", costmap_tolerance_);
    this->declare_parameter("external_rival_data_path", "");
    this->get_parameter("external_rival_data_path", external_rival_data_path_);

    RCLCPP_INFO(this->get_logger(), "\033[1;35m SystemCheck started, waiting for startup plan... \033[0m");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_pose_;
  nav_msgs::msg::Odometry::SharedPtr latest_pose_;
  rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Client<DockRobot>::SharedPtr dock_robot_client_;
  rclcpp::TimerBase::SharedPtr obstacle_timer_;
  bool running_;
  int costmap_tolerance_;
  std::string external_rival_data_path_;
  int obstacle_check_count_;

  void readyCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg == nullptr || running_)
      return;
    
    running_ = true;  // Set running state to true to prevent re-entrance

    // Wait for services and action servers
    while (!ready_srv_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for ReadySignal service...");
      if (!rclcpp::ok()) return;
    }
    while (!navigate_to_pose_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for navigate_to_pose action server...");
      if (!rclcpp::ok()) return;
    }
    while (!dock_robot_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for dock_robot action server...");
      if (!rclcpp::ok()) return;
    }

    // Start obstacle check timer if in obstacle
    if (inObstacle()) {
      RCLCPP_WARN(this->get_logger(), "Robot is in obstacles, waiting for manual action...");
      obstacle_check_count_ = 10;
      obstacle_timer_ = this->create_wall_timer(
        1s, std::bind(&SystemCheck::obstacleCheckTimer, this));
      return;
    }

    // All systems ready
    RCLCPP_INFO(this->get_logger(), "\033[1;32m All systems ready !!! \033[0m");
    sendReadySignal(3, 3);  // group = 3 (navigation), state = 3 (START)
  }

  void obstacleCheckTimer()
  {
    if (!inObstacle()) {
      obstacle_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "\033[1;32m All systems ready after obstacle cleared! \033[0m");
      sendReadySignal(3, 3);
      return;
    }

    if (obstacle_check_count_ > 0) {
      RCLCPP_INFO(this->get_logger(), "\033[1;35m Automatic shrinking process is going to activate after %d seconds... \033[0m", obstacle_check_count_);
      obstacle_check_count_--;
    } else {
      shrinkRivalRadius();
      // After shrinking, check again in 0.1s
      obstacle_timer_->cancel();
      obstacle_timer_ = this->create_wall_timer(
        100ms, std::bind(&SystemCheck::obstacleCheckTimer, this));
    }
  }

  void sendReadySignal(int group, int state)
  {
    auto request = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();
    request->group = group;
    request->state = state;

    ready_srv_client_->async_send_request(request,
      [this](rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m ReadySignal SUCCESS: group=%d \033[0m", response->group);
            running_ = true; // Set running state to true after successful signal
            return;
        } else {
            RCLCPP_WARN(this->get_logger(), "ReadySignal FAILED");
        }
      });

      running_ = false; // Reset running state after sending the signal
  }

  bool inObstacle() {
    if (!latest_costmap_ || !latest_pose_) {
        RCLCPP_WARN(this->get_logger(), "No costmap or pose received yet.");
        return false;
    }

    int mapX = static_cast<int>(latest_pose_->pose.pose.position.x * 100.0);
    int mapY = static_cast<int>(latest_pose_->pose.pose.position.y * 100.0);
    int width = latest_costmap_->info.width;
    int index = mapY * width + mapX;

    if (latest_costmap_->data[index] > costmap_tolerance_) {
        // RCLCPP_INFO(this->get_logger(), "Obstacle data is [%d]", latest_costmap_->data[index]);
        return true;
    }
    return false;
  }

  void shrinkRivalRadius() {
      try {
          YAML::Node config = YAML::LoadFile(external_rival_data_path_);
          if (config["nav_rival_parameters"] && config["nav_rival_parameters"]["rival_inscribed_radius"]) {
              double rival_inscribed_radius = config["nav_rival_parameters"]["rival_inscribed_radius"].as<double>();
              rival_inscribed_radius -= 0.01;
              config["nav_rival_parameters"]["rival_inscribed_radius"] = std::max(rival_inscribed_radius, 0.0);
              RCLCPP_INFO(this->get_logger(), "\033[1;35m Shrunk rival_inscribed_radius to %f \033[0m", config["nav_rival_parameters"]["rival_inscribed_radius"].as<double>());
          } else {
              RCLCPP_WARN(this->get_logger(), "rival_inscribed_radius not found in YAML file, cannot shrink automatically");
              return;
          }
          std::ofstream fout(external_rival_data_path_);
          fout << config;
          fout.close();
      } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "%s %s -> Failed to update YAML file, cannot shrink automatically", e.what(), external_rival_data_path_.c_str());
      }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemCheck>();
  // Use MultiThreadedExecutor for safety if you add more timers or callbacks
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  return 0;
}