// Copyright (c) 2024 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENNAV_DOCKING__CONTROLLER_HPP_
#define OPENNAV_DOCKING__CONTROLLER_HPP_


#include <algorithm>
#include <string>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include "std_msgs/msg/string.hpp"

namespace opennav_docking
{
class RobotState {
  public:
    RobotState(double x, double y, double theta);
    RobotState() {}
    RobotState(const RobotState & other) = default;
    double x_;
    double y_;
    double theta_;
    // Eigen::Vector3d getVector();
    double distanceTo(RobotState);
    // Eigen::Vector2d orientationUnitVec() const { return Eigen::Vector2d(cos(theta_), sin(theta_)); }
    void operator=(const RobotState& rhs) {
        this->x_ = rhs.x_;
        this->y_ = rhs.y_;
        this->theta_ = rhs.theta_;
  }
};

// State machine for velocity control
enum class VelocityState {
  ACCELERATION,
  CONSTANT,
  DECELERATION
};

/**
 * @class opennav_docking::Controller
 * @brief Custom controller for approaching a dock target
 */
class Controller
{
  public:
    /**
     * @brief Create a controller instance. Configure ROS 2 parameters.
     */
    explicit Controller(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

    /**
     * @brief Compute a velocity command using control law.
     * @param pose Target pose, in robot centric coordinates.
     * @param cmd Command velocity.
     * @param backward If true, robot will drive backwards to goal.
     * @returns True if command is valid, false otherwise.
     */
    bool computeVelocityCommand(
      const geometry_msgs::msg::Pose & target, geometry_msgs::msg::Twist & cmd,
      bool backward = false);
    
    /**
     * @brief Compute if rival is on the way.
     * @param target Target pose, in global coordinates.
     * @returns True if rival is on the way, false otherwise.
     */
    bool computeIfNeedStop(const geometry_msgs::msg::Pose & target);

    /**
     * @brief Set the total distance for velocity control & Set velocity state to acceleration.
     * @param target Target pose, in robot centric coordinates.
     * @note Please call this function before calling loop for computeVelocityCommand.
     */
    void velocityInit(const geometry_msgs::msg::Pose & target);

    void posetoRobotState(geometry_msgs::msg::Pose pose, RobotState &state);
    double getGoalAngle(double ang_diff);
    RobotState getLookAheadPoint(RobotState cur_pose, std::vector<RobotState> &path, double look_ahead_distance);
    RobotState globalTolocal(RobotState cur_pose, RobotState goal);

  protected:
    // Node configuration
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("CustomController")};
    
    // Parameter configuration
    std::string param_name_ = "Ordinary";
    std::vector<std::string> profiles_;
    void declareAllControlParams();
    void updateParams();
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_controller_selector_sub_;

    // Parameters from the config file
    double max_linear_vel_, min_linear_vel_;
    double max_angular_vel_, min_angular_vel_;
    double max_linear_acc_, max_angular_acc_;

    double initial_decel_speed_;
    double vel_error_sum_;
    double linear_ki_accel_vel_, linear_kp_accel_vel_;
    double linear_kp_decel_dis_, linear_kp_decel_vel_;
    double angular_kp_;
    double look_ahead_distance_;
    double final_goal_angle_;

    // see if need to stop
    double stop_degree_;
    double rival_radius_;

    // Variables
    std::vector<RobotState> global_path_;
    std::vector<RobotState> vector_global_path_;
    RobotState robot_pose_;
    RobotState rival_pose_;
    RobotState local_goal_;
    double total_distance_;
    double deceleration_distance_;
    double reserved_distance_;

    // Robot pose subscibtion
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rival_pose_sub_;
    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr robot_pose);
    void rivalPoseCallback(const nav_msgs::msg::Odometry::SharedPtr rival_pose);

    // Local goal publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
    void publishLocalGoal();

    /** 
    * @brief Reset the state of the controller to acceleration 
    */
    void ResetState() { 
      state_x_ = VelocityState::ACCELERATION; 
      state_y_ = VelocityState::ACCELERATION;
      vel_error_sum_ = 0.0;
    }

    /**
     * @brief Genarate linear velocity command with speed control
     * @param vel velocity
     * @param remaining_distance remaining distance to the goal
     * @param total_distance total distance to the goal
     */
    double ExtractVelocity(const double & velocity, const double & remaining_distance, VelocityState & state);

    // Velocity control functions
    void Acceleration(double & vel, const double & remaining_distance, VelocityState & state);
    void ConstantVelocity(double & vel, const double & remaining_distance, VelocityState & state);
    void Deceleration(double & vel, const double & remaining_distance, VelocityState & state);

    VelocityState state_x_;
    VelocityState state_y_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_HPP_
