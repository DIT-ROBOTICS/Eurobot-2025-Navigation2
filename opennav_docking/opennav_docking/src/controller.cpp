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

#include <algorithm>
#include <string>
#include <memory>

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


using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using rcl_interfaces::msg::ParameterType;

namespace opennav_docking
{

Controller::Controller(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) {
    // Declare parameters if not declared
    declare_parameter_if_not_declared(node, "max_linear_vel", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, "min_linear_vel", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(node, "max_angular_vel", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, "min_angular_vel", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(node, "max_linear_acc", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, "max_angular_acc", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, "angular_kp", rclcpp::ParameterValue(4.0));
    declare_parameter_if_not_declared(node, "look_ahead_distance", rclcpp::ParameterValue(1.0));

    // Get parameters from the config file
    node->get_parameter("max_linear_vel", max_linear_vel_);
    node->get_parameter("min_linear_vel", min_linear_vel_);
    node->get_parameter("max_angular_vel", max_angular_vel_);
    node->get_parameter("min_angular_vel", min_angular_vel_);
    node->get_parameter("max_linear_acc", max_linear_acc_);
    node->get_parameter("max_angular_acc", max_angular_acc_);
    node->get_parameter("angular_kp", angular_kp_);
    node->get_parameter("look_ahead_distance", look_ahead_distance_);

    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Subscribe to the robot's pose
    robot_pose_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        rclcpp::QoS(10).durability_volatile(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_pose_.x_ = (*msg).pose.pose.position.x;
            robot_pose_.y_ = (*msg).pose.pose.position.y;
        });

    // Publish the local goal
    local_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "local_goal",
        rclcpp::QoS(10));
}

RobotState::RobotState(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos) {
    return sqrt(pow(x_ - pos.x_, 2) + pow(y_ - pos.y_, 2));
}

void Controller::posetoRobotState(geometry_msgs::msg::Pose pose, RobotState &state) {
    state.x_ = pose.position.x;
    state.y_ = pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double pitch, row, yaw;
    qt.getRPY(pitch, row, yaw);
    
    state.theta_ = yaw;
}

RobotState Controller::globalTolocal(RobotState cur_pose, RobotState goal) {
    RobotState local_goal;
    local_goal.x_ = (goal.x_ - cur_pose.x_) * cos(cur_pose.theta_) + (goal.y_ - cur_pose.y_) * sin(cur_pose.theta_);
    local_goal.y_ = -(goal.x_ - cur_pose.x_) * sin(cur_pose.theta_) + (goal.y_ - cur_pose.y_) * cos(cur_pose.theta_);

    return local_goal;
}

double Controller::getGoalAngle(double cur_angle, double goal_angle) {
    double ang_diff_ = goal_angle - cur_angle;
    double angular_max_vel_ = 2.0;
    double angle_vel_ = 0.0;

    if(cur_angle >= 0 && goal_angle >= 0) {
        if(ang_diff_ >= 0) angle_vel_ = std::min((ang_diff_ * angular_kp_), angular_max_vel_);
        else angle_vel_ = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
    } else if(cur_angle < 0 && goal_angle < 0) {
        if(ang_diff_ >= 0) angle_vel_ = std::min((ang_diff_ * angular_kp_), angular_max_vel_);
        else angle_vel_ = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
    } else if(cur_angle < 0 && goal_angle >= 0) {
        if((fabs(cur_angle) + goal_angle) >= M_PI) angle_vel_ = std::max((-ang_diff_ * angular_kp_), -angular_max_vel_);
        else angle_vel_ = std::min((ang_diff_ * angular_kp_), angular_max_vel_); 
    } else {
        if((cur_angle + fabs(goal_angle)) <= M_PI) angle_vel_ = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
        else angle_vel_ = std::min((-ang_diff_ * angular_kp_), angular_max_vel_);
    }

    return angle_vel_;
}

bool Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & target, geometry_msgs::msg::Twist & cmd, bool /*backward*/) {
        
    double global_distance = sqrt(pow(target.position.x, 2) + pow(target.position.y, 2));
    // ? Why
    // local_goal_ = globalTolocal(robot_pose_, local_goal_);
    double local_angle = atan2(target.position.y, target.position.x);
    
    publishLocalGoal();

    cmd.linear.x = std::min(global_distance * 1.5, max_linear_vel_) * cos(local_angle);
    cmd.linear.y = std::min(global_distance * 1.5, max_linear_vel_) * sin(local_angle);
    cmd.angular.z = getGoalAngle(robot_pose_.theta_, final_goal_angle_);
    return true;
}

void Controller::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr robot_pose) {
    posetoRobotState(robot_pose->pose.pose, robot_pose_); 
}

void Controller::publishLocalGoal() {
    geometry_msgs::msg::PoseStamped local_goal;
    local_goal.header.frame_id = "map";
    local_goal.header.stamp = clock_->now();
    local_goal.pose.position.x = robot_pose_.x_;
    local_goal.pose.position.y = robot_pose_.y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, local_goal_.theta_);
    local_goal.pose.orientation.x = q.x();
    local_goal.pose.orientation.y = q.y();
    local_goal.pose.orientation.z = q.z();
    local_goal.pose.orientation.w = q.w();
    local_goal_pub_->publish(local_goal);
}

}  // namespace opennav_docking
