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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "opennav_docking/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "angles/angles.h"

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
    declare_parameter_if_not_declared(node, "max_linear_vel", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, "min_linear_vel", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(node, "max_angular_vel", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, "min_angular_vel", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(node, "max_linear_acc", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, "max_angular_acc", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, "angular_kp", rclcpp::ParameterValue(4.0));
    declare_parameter_if_not_declared(node, "look_ahead_distance", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, "yaw_goal_tolerance", rclcpp::ParameterValue(0.01));
    declare_parameter_if_not_declared(node, "xy_goal_tolerance_sq", rclcpp::ParameterValue(0.004));

    // Get parameters from the config file
    node->get_parameter("max_linear_vel", max_linear_vel_);
    node->get_parameter("min_linear_vel", min_linear_vel_);
    node->get_parameter("max_angular_vel", max_angular_vel_);
    node->get_parameter("min_angular_vel", min_angular_vel_);
    node->get_parameter("max_linear_acc", max_linear_acc_);
    node->get_parameter("max_angular_acc", max_angular_acc_);
    node->get_parameter("angular_kp", angular_kp_);
    node->get_parameter("look_ahead_distance", look_ahead_distance_);
    node->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
    node->get_parameter("xy_goal_tolerance_sq", xy_goal_tolerance_sq_);

    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Subscribe to the rival's pose
    robot_pose_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 100, std::bind(&Controller::robotPoseCallback, this, std::placeholders::_1));
}

RobotState::RobotState(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos) {
    return sqrt(pow(x_ - pos.x_, 2) + pow(y_ - pos.y_, 2));
}

bool isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &) {
    double dx = query_pose.position.x - goal_pose.position.x;
    double dy = query_pose.position.y - goal_pose.position.y;

    if(dx * dx + dy * dy > xy_goal_tolerance_sq_) {
        return false;
    }

    double dyaw = angles::shortest_angular_distance(
        tf2::getYaw(query_pose.orientation),
        tf2::getYaw(goal_pose.orientation));

    return fabs(dyaw) < yaw_goal_tolerance_;
}

void Controller::dividePath(const geometry_msgs::msg::Pose & target) {
    // Divide the path into segments
    // Pose -> Segment -> Target
    RobotState point_buffer;

    global_path_.clear();

    int distance = hypot(target.position.x - robot_pose_.x_, target.position.y - robot_pose_.y_);
    if(distance < 1) {
        posetoRobotState(target, point_buffer);
        global_path_.push_back(point_buffer);
        return;
    } 
    for(int i=0; i<=distance; i++) {
        point_buffer.x_ = robot_pose_.x_ + i * (target.position.x - robot_pose_.x_) / distance;
        point_buffer.y_ = robot_pose_.y_ + i * (target.position.y - robot_pose_.y_) / distance;
        point_buffer.theta_ = tf2::getYaw(target.orientation);

        global_path_.push_back(point_buffer);
    }
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

RobotState Controller::getLookAheadPoint(
  RobotState cur_pose, std::vector<RobotState> &path, double look_ahead_distance) {
    if(path.empty()) {
        // RCLCPP_INFO(logger_, "[%s] Path is empty", plugin_name_.c_str());
        return cur_pose;
    }
    
    RobotState local_goal;
    
    int nearest_index = 0;
    int next_index = 0;

    for(int i=path.size()-1; i>=0; --i) {
        if(cur_pose.distanceTo(path[i]) <= look_ahead_distance) {
            next_index = i;
            break;
        }
    }

    if (next_index == 0) next_index = path.size()-1;

    if (next_index < path.size()-1) {
        next_index = next_index+1;
    }

    local_goal.x_ = cur_pose.x_ + (path[next_index].x_ - cur_pose.x_)*(look_ahead_distance/cur_pose.distanceTo(path[next_index]));
    local_goal.y_ = cur_pose.y_ + (path[next_index].y_ - cur_pose.y_)*(look_ahead_distance/cur_pose.distanceTo(path[next_index]));
    local_goal.theta_ = path[next_index].theta_;

    if (cur_pose.distanceTo(path.back()) < look_ahead_distance + 0.01)
        local_goal = path.back();

    if (local_goal.distanceTo(path.back()) < 0.005) {
        local_goal = path.back();
    }
    
    // for rviz visualization
    geometry_msgs::msg::PoseStamped pos_msg;
    pos_msg.header.frame_id = "map";
    pos_msg.header.stamp = clock_->now();
    pos_msg.pose.position.x = local_goal.x_;
    pos_msg.pose.position.y = local_goal.y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, local_goal.theta_);
    pos_msg.pose.orientation.x = q.x();
    pos_msg.pose.orientation.y = q.y();
    pos_msg.pose.orientation.z = q.z();
    pos_msg.pose.orientation.w = q.w();
    local_goal_pub_->publish(pos_msg);
    
    return local_goal;
}

double Controller::getGoalAngle(double cur_angle, double goal_angle) {
    double ang_diff_ = goal_angle - cur_angle;
    double angular_max_vel_ = 2.0;
    double angle_vel_ = 0.0;
    double angular_kp_ = 4.0;

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
    dividePath(target);

    // cmd_vel
    cmd.header.frame_id = pose.header.frame_id;
    cmd.header.stamp = clock_->now();

    if(!isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)){
        local_goal_ = getLookAheadPoint(robot_pose_, vector_global_path_, look_ahead_distance_);
        
        double global_distance = sqrt(pow(global_plan_.poses.back().pose.position.x - robot_pose_.x_, 2) + pow(global_plan_.poses.back().pose.position.y - robot_pose_.y_, 2));
        local_goal_ = globalTolocal(robot_pose_, local_goal_);
        double local_angle = atan2(local_goal_.y_, local_goal_.x_);
        
        cmd.twist.linear.x = std::min(global_distance * 1.5, max_linear_vel_) * cos(local_angle);
        cmd.twist.linear.y = std::min(global_distance * 1.5, max_linear_vel_) * sin(local_angle);
        cmd.twist.angular.z = getGoalAngle(robot_pose_.theta_, final_goal_angle_);
        return true;

    } else if(fabs(final_goal_angle_ - robot_pose_.theta_) > 0.01) {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = getGoalAngle(robot_pose_.theta_, final_goal_angle_);
        return true;

    } else {
        // RCLCPP_INFO(logger_, "[%s] Goal reached", plugin_name_.c_str());
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;
        return true;
    }
}

void Controller::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr robot_pose) {
    posetoRobotState(robot_pose.pose.pose, robot_pose_); 
}

}  // namespace opennav_docking
