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
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
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
    // Initialize node
    node_ = node;

    // Declare & get controller profiles
    node_->declare_parameter("controllers", rclcpp::ParameterValue(std::vector<std::string>()));
    node_->get_parameter("controllers", profiles_);

    // Declare parameters if not declared
    declareAllControlParams();

    // Get parameters from the config file
    updateParams();

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
    
    // Subscribe to the rival's pose
    rival_pose_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/rival/final_pose",
        rclcpp::QoS(10).durability_volatile(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            rival_pose_.x_ = (*msg).pose.pose.position.x;
            rival_pose_.y_ = (*msg).pose.pose.position.y;
        });

    // Publish the local goal
    local_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "local_goal",
        rclcpp::QoS(10));

    // Subscribe to dock controller selector
    dock_controller_selector_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/dock_controller_type",
        rclcpp::QoS(10).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            param_name_ = msg->data;
            updateParams();
        });
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

double Controller::getGoalAngle(double ang_diff) {
    double angular_max_vel_ = 2.0;
    double angle_vel_ = 0.0;

    if(ang_diff >= 0) {
        angle_vel_ = std::min((ang_diff * angular_kp_), angular_max_vel_);
    } else {
        angle_vel_ = std::max((ang_diff * angular_kp_), -angular_max_vel_);
    }

    return angle_vel_;
}

// ** Has been called in docking_server.cpp
void Controller::velocityInit(const geometry_msgs::msg::Pose & target) {
    total_distance_ = sqrt(pow(target.position.x - robot_pose_.x_, 2) + pow(target.position.y - robot_pose_.y_, 2));  // Target is in global frame
    // RCLCPP_INFO(logger_, "Total distance: %f", total_distance_);
    ResetState();
}

bool Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & target, geometry_msgs::msg::Twist & cmd, bool /*backward*/) {
    
    double global_distance = sqrt(pow(target.position.x, 2) + pow(target.position.y, 2));   // Target is in base_link frame
    double global_angle = tf2::getYaw(target.orientation);
    // ? Why
    // local_goal_ = globalTolocal(robot_pose_, local_goal_);
    double local_angle = atan2(target.position.y, target.position.x);

    publishLocalGoal();
    
    cmd.linear.x = ExtractVelocity(cmd.linear.x, global_distance, state_x_) * cos(local_angle);
    cmd.linear.y = ExtractVelocity(cmd.linear.y, global_distance, state_y_) * sin(local_angle);
    cmd.angular.z = getGoalAngle(global_angle);
    
    return true;
}

bool Controller::computeIfNeedStop(const geometry_msgs::msg::Pose & target) {
    // Calculate the vector from the robot to the target
    double target_dx = target.position.x - robot_pose_.x_;
    double target_dy = target.position.y - robot_pose_.y_;
    double target_distance = std::sqrt(target_dx * target_dx + target_dy * target_dy);
    
    // Calculate the angle to the target
    double target_angle = std::atan2(target_dy, target_dx);
    
    // Calculate the vector from the robot to the rival
    double rival_dx = rival_pose_.x_ - robot_pose_.x_;
    double rival_dy = rival_pose_.y_ - robot_pose_.y_;
    double rival_distance = std::sqrt(rival_dx * rival_dx + rival_dy * rival_dy) - rival_radius_;  // 0.44 is the radius of the rival
    
    // Calculate the angle to the rival
    double rival_angle = std::atan2(rival_dy, rival_dx);
    
    // Calculate the angle difference
    double angle_diff = angles::shortest_angular_distance(target_angle, rival_angle);
    
    // if (std::fabs(angle_diff) <= angles::from_degrees(stop_degree_ / 2.0) && rival_distance <= target_distance) {
    //     RCLCPP_INFO(logger_, "Stop the robot, because target_angle: %f, rival_angle: %f,angle_diff: %f, rival_distance: %f, target_distance: %f", target_angle, rival_angle,angle_diff, rival_distance, target_distance);
    // }
    
    // Check if the rival is within the stop degree from the robot to the target
    return (std::fabs(angle_diff) <= angles::from_degrees(stop_degree_ / 2.0) && rival_distance <= target_distance);

    // RCLCPP_INFO(logger_, "target pose x: %f, y: %f", target.position.x, target.position.y);
    // RCLCPP_INFO(logger_, "Don't Stop the robot, because target_angle: %f, rival_angle: %f,angle_diff: %f, rival_distance: %f, target_distance: %f",target_angle, rival_angle, angle_diff, rival_distance, target_distance);
}

double Controller::ExtractVelocity(const double & velocity, const double & remaining_distance, VelocityState & state) {
    double vel = velocity;

    switch (state) {
        case VelocityState::ACCELERATION:
            Acceleration(vel, remaining_distance, state);
            // RCLCPP_INFO_ONCE(logger_, "Accelerating");
            break;
        case VelocityState::CONSTANT:
            ConstantVelocity(vel, remaining_distance, state);
            // RCLCPP_INFO_ONCE(logger_, "Constant velocity");
            break;
        case VelocityState::DECELERATION:
            Deceleration(vel, remaining_distance, state);
            // RCLCPP_INFO_ONCE(logger_, "Decelerating");
            break;
    }

    return vel;
}

void Controller::Acceleration(double & vel, const double & remaining_distance, VelocityState & state) {
    vel += linear_kp_accel_vel_ * fabs(vel - max_linear_vel_) + linear_ki_accel_vel_ * vel_error_sum_;
    vel = std::min(vel, max_linear_vel_);
    // vel = std::max(vel, min_linear_vel_);

    vel_error_sum_ += fabs(vel - max_linear_vel_);
    vel_error_sum_ = std::min(vel_error_sum_, 3.0);

    if(vel >= max_linear_vel_) {
        state = VelocityState::CONSTANT;
    } else if(remaining_distance < deceleration_distance_) {
        initial_decel_speed_ = vel;
        state = VelocityState::DECELERATION;
    }
}

void Controller::ConstantVelocity(double & vel, const double & remaining_distance, VelocityState & state) {
    vel = max_linear_vel_;

    if(remaining_distance < deceleration_distance_) {
        initial_decel_speed_ = vel;
        state = VelocityState::DECELERATION;
    }
}

void Controller::Deceleration(double & vel, const double & remaining_distance, VelocityState & /*state*/) {
    vel = std::min(linear_kp_decel_dis_ * remaining_distance, initial_decel_speed_);
    vel = std::min(vel, max_linear_vel_);
    vel = std::max(vel, min_linear_vel_);
    if(remaining_distance < reserved_distance_) {
        vel = min_linear_vel_;
    }
}

void Controller::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr robot_pose) {
    posetoRobotState(robot_pose->pose.pose, robot_pose_); 
}

void Controller::rivalPoseCallback(const nav_msgs::msg::Odometry::SharedPtr rival_pose) {
    posetoRobotState(rival_pose->pose.pose, rival_pose_);
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

void Controller::declareAllControlParams()
{
    std::vector<std::pair<std::string, rclcpp::ParameterValue>> params = {
        {"max_linear_vel", rclcpp::ParameterValue(0.5)},
        {"min_linear_vel", rclcpp::ParameterValue(0.1)},
        {"max_angular_vel", rclcpp::ParameterValue(3.0)},
        {"min_angular_vel", rclcpp::ParameterValue(0.0)},
        {"max_linear_acc", rclcpp::ParameterValue(0.3)},
        {"max_angular_acc", rclcpp::ParameterValue(1.0)},
        {"linear_ki_accel_vel", rclcpp::ParameterValue(0.7)},
        {"linear_kp_accel_vel", rclcpp::ParameterValue(0.5)},
        {"linear_kp_decel_dis", rclcpp::ParameterValue(3.0)},
        {"linear_kp_decel_vel", rclcpp::ParameterValue(0.9)},
        {"angular_kp", rclcpp::ParameterValue(4.0)},
        {"look_ahead_distance", rclcpp::ParameterValue(1.0)},
        {"deceleration_distance", rclcpp::ParameterValue(0.1)},
        {"reserved_distance", rclcpp::ParameterValue(0.03)},
        {"stop_degree", rclcpp::ParameterValue(45.0)},
        {"rival_radius", rclcpp::ParameterValue(0.44)},
    };

    for (const auto& profile : profiles_)
    {
        for (const auto& [name, default_value] : params)
        {
            const std::string full_name = profile + "." + name;
            if (!node_->has_parameter(full_name))
            {
                node_->declare_parameter(full_name, default_value);
            }
        }
    }
}

void Controller::updateParams() {
    node_->get_parameter(param_name_ + ".max_linear_vel", max_linear_vel_);
    node_->get_parameter(param_name_ + ".min_linear_vel", min_linear_vel_);
    node_->get_parameter(param_name_ + ".max_angular_vel", max_angular_vel_);
    node_->get_parameter(param_name_ + ".min_angular_vel", min_angular_vel_);
    node_->get_parameter(param_name_ + ".max_linear_acc", max_linear_acc_);
    node_->get_parameter(param_name_ + ".max_angular_acc", max_angular_acc_);
    node_->get_parameter(param_name_ + ".linear_ki_accel_vel", linear_ki_accel_vel_);
    node_->get_parameter(param_name_ + ".linear_kp_accel_vel", linear_kp_accel_vel_);
    node_->get_parameter(param_name_ + ".linear_kp_decel_dis", linear_kp_decel_dis_);
    node_->get_parameter(param_name_ + ".linear_kp_decel_vel", linear_kp_decel_vel_);
    node_->get_parameter(param_name_ + ".angular_kp", angular_kp_);
    node_->get_parameter(param_name_ + ".look_ahead_distance", look_ahead_distance_);
    node_->get_parameter(param_name_ + ".deceleration_distance", deceleration_distance_);
    node_->get_parameter(param_name_ + ".reserved_distance", reserved_distance_);
    node_->get_parameter(param_name_ + ".stop_degree", stop_degree_);
    node_->get_parameter(param_name_ + ".rival_radius", rival_radius_);
}

}  // namespace opennav_docking
