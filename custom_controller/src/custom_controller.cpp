#include <algorithm>
#include <string>
#include <memory>

#include "pluginlib/class_list_macros.hpp"
#include "custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

PLUGINLIB_EXPORT_CLASS(custom_controller::CustomController, nav2_core::Controller)

namespace custom_controller
{

// Configure the controller

void CustomController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    auto node = parent.lock();
    speed_test = 0.0;
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

   
    // Declare parameters if not declared
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_angular_vel", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_acc", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_acc", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.01));
    declare_parameter_if_not_declared(node, plugin_name_ + ".angular_kp", rclcpp::ParameterValue(4.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".look_ahead_distance", rclcpp::ParameterValue(1.0));
    // Get parameters from the config file
    node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
    node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    node->get_parameter(plugin_name_ + ".min_angular_vel", min_angular_vel_);
    node->get_parameter(plugin_name_ + ".max_linear_acc", max_linear_acc_);
    node->get_parameter(plugin_name_ + ".max_angular_acc", max_angular_acc_);
    node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
    node->get_parameter(plugin_name_ + ".angular_kp", angular_kp_);
    node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
    double transform_tolerance;
    
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
}

// Lifecycle methods
void CustomController::cleanup(){
    RCLCPP_INFO(logger_, "[%s] Cleaning up controller", plugin_name_.c_str());
}
void CustomController::activate(){
    RCLCPP_INFO(logger_, "[%s] Activating controller", plugin_name_.c_str());
}
void CustomController::deactivate(){
    RCLCPP_INFO(logger_, "[%s] Deactivating controller", plugin_name_.c_str());
}

// void CustomController::setSpeedLimit(double speed_limit, double speed_limit_yaw){
//   speed_limit_ = speed_limit;
//   speed_limit_yaw_ = speed_limit_yaw;
// }

// Get the global plan with transformed poses
void CustomController::setPlan(const nav_msgs::msg::Path & path)
{
    RCLCPP_INFO(logger_, "Received a new plan");
    global_plan_ = path;
    RCLCPP_INFO(logger_, "global_plan_ final angle = [%lf]", global_plan_.poses.back().pose.orientation.z);
}
RobotState::RobotState(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos) {
    return sqrt(pow(x_ - pos.x_, 2) + pow(y_ - pos.y_, 2));
}
void CustomController::posetoRobotState(geometry_msgs::msg::Pose pose, RobotState &state) {
    state.x_ = pose.position.x;
    state.y_ = pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double pitch, row, yaw;
    qt.getRPY(pitch, row, yaw);
    
    state.theta_ = yaw;
}

void CustomController::pathToVector(nav_msgs::msg::Path path, std::vector<RobotState> &vector_path) {
    for (int i = 0; i < path.poses.size(); i++) {
        RobotState state;
        posetoRobotState(path.poses[i].pose, state);
        vector_path.push_back(state);
    }
}
RobotState CustomController::globalTOlocal(RobotState cur_pose, RobotState goal) {
    RobotState local_goal;
    local_goal.x_ = (goal.x_ - cur_pose.x_) * cos(cur_pose.theta_) + (goal.y_ - cur_pose.y_) * sin(cur_pose.theta_);
    local_goal.y_ = -(goal.x_ - cur_pose.x_) * sin(cur_pose.theta_) + (goal.y_ - cur_pose.y_) * cos(cur_pose.theta_);
    // local_goal.theta_ = goal.theta_ - cur_pose.theta_;
    //RCLCPP_INFO(logger_, "local_goal is [%lf] [%lf] [%lf]", local_goal.x_, local_goal.y_, local_goal.theta_);
    return local_goal;
}
RobotState CustomController::getLookAheadPoint(
    RobotState cur_pose,
    std::vector<RobotState> &path, 
    double look_ahead_distance)
{
    
    if (path.empty()) {
        RCLCPP_INFO(logger_, "[%s] Path is empty", plugin_name_.c_str());
        return cur_pose;
    }
    
    RobotState local_goal;
    
    int nearest_index = 0;
    int next_index = 0;
    
    //RCLCPP_INFO(logger_, "path size [%d]", path.size());
    for(int i=path.size()-1; i>=0; --i){
        if(cur_pose.distanceTo(path[i]) <= look_ahead_distance) {
            next_index = i;
            //RCLCPP_INFO(logger_, "next_index [%d]", next_index);
            break;
        }
    }
    if (next_index == 0) next_index = path.size()-1;

    if (next_index < path.size()-1) {
        next_index = next_index+1;
    }
    // while (next_index < path.size()-1 && cur_pose.distanceTo(path[next_index]) < look_ahead_distance) {
    //     next_index++;
    // }
    // if (next_index == 0) next_index = path.size()-1;
    //RCLCPP_INFO(logger_, "current pose is [%lf] [%lf]", cur_pose.x_, cur_pose.y_);

    local_goal.x_ = cur_pose.x_ + (path[next_index].x_ - cur_pose.x_)*(look_ahead_distance/cur_pose.distanceTo(path[next_index]));
    local_goal.y_ = cur_pose.y_ + (path[next_index].y_ - cur_pose.y_)*(look_ahead_distance/cur_pose.distanceTo(path[next_index]));
    local_goal.theta_ = path[next_index].theta_;

    if (cur_pose.distanceTo(path.back()) < look_ahead_distance + 0.01)
        local_goal = path.back();

    if (local_goal.distanceTo(path.back()) < 0.005) {
        local_goal = path.back();
        // is_local_goal_final_reached_ = true;
    }
    // ROS_INFO_STREAM("[Path Executor]: next_index: " << next_index);    
    // // for rviz visualization
    // geometry_msgs::PoseStamped pos_msg;
    // pos_msg.header.frame_id = frame_;
    // pos_msg.header.stamp = ros::Time::now();
    // pos_msg.pose.position.x = local_goal.x_;
    // pos_msg.pose.position.y = local_goal.y_;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, local_goal.theta_);
    // pos_msg.pose.orientation.x = q.x();
    // pos_msg.pose.orientation.y = q.y();
    // pos_msg.pose.orientation.z = q.z();
    // pos_msg.pose.orientation.w = q.w();
    // local_goal_pub_.publish(pos_msg);
    
    //RCLCPP_INFO(logger_, "local_goal is [%lf] [%lf]", local_goal.x_, local_goal.y_);
    // RCLCPP_INFO(logger_, "angle = [%lf]", cur_pose.theta_);
    path.clear();
    return local_goal;

    
}
geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
    posetoRobotState(pose.pose, cur_pose_);
    pathToVector(global_plan_, vector_global_path_);
    // cmd_vel
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    if(!goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)){
        
        //RCLCPP_INFO(logger_, "look_ahead_distance is [%lf]", look_ahead_distance_);
        final_goal_angle_ = vector_global_path_[vector_global_path_.size()-1].theta_ - cur_pose_.theta_;
        //RCLCPP_INFO(logger_, "final goal angle raw is [%lf]", global_plan_.poses.back().pose.orientation.z);
        local_goal_ = getLookAheadPoint(cur_pose_, vector_global_path_, look_ahead_distance_); 
        double global_distance = sqrt(pow(global_plan_.poses.back().pose.position.x - cur_pose_.x_, 2) + pow(global_plan_.poses.back().pose.position.y - cur_pose_.y_, 2));
        local_goal_ = globalTOlocal(cur_pose_, local_goal_);
        double local_angle = atan2(local_goal_.y_, local_goal_.x_);
        double local_distance = sqrt(pow(local_goal_.x_ - cur_pose_.x_, 2) + pow(local_goal_.y_ - cur_pose_.y_, 2));
        
        //RCLCPP_INFO(logger_, "final goal angle is [%lf]", vector_global_path_[vector_global_path_.size()-1].theta_);
        RCLCPP_INFO(logger_, "cur_pose angle is [%lf]", cur_pose_.theta_);
        cmd_vel.twist.linear.x = std::min(global_distance * 1.5, max_linear_vel_) * cos(local_angle);
        cmd_vel.twist.linear.y = std::min(global_distance * 1.5, max_linear_vel_) * sin(local_angle);
        cmd_vel.twist.angular.z = (final_goal_angle - cur_pose_.theta_)  * 5;
        RCLCPP_INFO(logger_, "cmd_vel is [%lf] [%lf] [%lf]", cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
        //RCLCPP_INFO(logger_, "local_angle is [%lf]", local_angle);
        
        
        // Spin
        //     double dyaw = angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation), tf2::getYaw(global_plan_.poses.back().pose.orientation));
        //     if (fabs(dyaw) > yaw_goal_tolerance_) {
        //         yaw_debounce_counter_ = 0;
        //     } else {
        //         yaw_debounce_counter_++;
        //     }

        //     if(yaw_debounce_counter_ > 5){
        //         RCLCPP_INFO(logger_, "[%s] Orientation reached", plugin_name_.c_str());
        //         cmd_vel.twist.angular.z = 0.0;
        //     } else {
        //         cmd_vel.twist.angular.z = 1.0;
        //     }

        //     // Move
        //     // cmd_vel.twist.linear.x = 0.1;
        //     // cmd_vel.twist.linear.y = 0.1;
        // }
        // else {
        //     RCLCPP_INFO(logger_, "[%s] Goal reached", plugin_name_.c_str());
        //     cmd_vel.twist.linear.x = 0.0;
        //     cmd_vel.twist.linear.y = 0.0;
        //     cmd_vel.twist.angular.z = 0.0;
        // }
        // cmd_vel.twist.linear.x = 0.0;
        // cmd_vel.twist.linear.y = 0.0;
        // cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }
    else{
        RCLCPP_INFO(logger_, "[%s] Goal reached", plugin_name_.c_str());
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }
}
void CustomController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
    return;
//   if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
//     // Restore default value
//     desired_linear_vel_ = base_desired_linear_vel_;
//   } else {
//     if (percentage) {
//       // Speed limit is expressed in % from maximum speed of robot
//       desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
//     } else {
//       // Speed limit is expressed in absolute value
//       desired_linear_vel_ = speed_limit;
//     }
//   }
}

}  // namespace custom_controller

PLUGINLIB_EXPORT_CLASS(custom_controller::CustomController, nav2_core::Controller)