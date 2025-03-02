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
#include "opennav_docking/controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

    void dividePath(const geometry_msgs::msg::Pose & target);
    void posetoRobotState(geometry_msgs::msg::Pose pose, RobotState &state);
    double getGoalAngle(double cur_pose, double goal);
    RobotState getLookAheadPoint(RobotState cur_pose, std::vector<RobotState> &path, double look_ahead_distance);
    RobotState globalTolocal(RobotState cur_pose, RobotState goal);
    bool isGoalReached(RobotState& robot_pose_, const geometry_msgs::msg::Pose &target, double xy_goal_tolerance);

  protected:
    // Node configuration
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("CustomController")};

    // Parameters from the config file
    double max_linear_vel_, min_linear_vel_;
    double max_angular_vel_, min_angular_vel_;
    double max_linear_acc_, max_angular_acc_;

    double angular_kp_;
    double look_ahead_distance_;
    double final_goal_angle_;

    double xy_goal_tolerance_sq_, yaw_goal_tolerance_;

    // Variables
    std::vector<RobotState> global_path_;
    std::vector<RobotState> vector_global_path_;
    RobotState robot_pose_;
    RobotState local_goal_;

    // Robot pose subscibtion
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr robot_pose);

    // Local goal publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
    void publishLocalGoal();
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_HPP_
