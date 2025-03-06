#include <algorithm>
#include <string>
#include <memory>

#include "pluginlib/class_list_macros.hpp"
#include "custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp" // For raw costmap data
#include "geometry_msgs/msg/pose_stamped.hpp"

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
    update_plan_ = true;
    isObstacleExist_ = false;
    costmap_subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap",  // Replace with your actual costmap topic
        rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            // Store the latest costmap
            latest_costmap_ = msg;
            // RCLCPP_INFO(logger_, "Received costmap data.");
        });

    rival_pose_subscription_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rival_pose",  // Replace with your actual rival pose topic
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            rival_pose_ = *msg;
        });

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 5);
    check_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("check_goal", 5);

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
    global_path_pub_.reset();
    check_goal_pub_.reset();
}
void CustomController::activate(){
    RCLCPP_INFO(logger_, "[%s] Activating controller", plugin_name_.c_str());
    global_path_pub_->on_activate();
    check_goal_pub_->on_activate();

}
void CustomController::deactivate(){
    RCLCPP_INFO(logger_, "[%s] Deactivating controller", plugin_name_.c_str());
    global_path_pub_->on_deactivate();
    check_goal_pub_->on_deactivate();
}

// void CustomController::setSpeedLimit(double speed_limit, double speed_limit_yaw){
//   speed_limit_ = speed_limit;
//   speed_limit_yaw_ = speed_limit_yaw;
// }

// Get the global plan with transformed poses
void CustomController::setPlan(const nav_msgs::msg::Path & path)
{
    if(!update_plan_){
        return;
    }
    
    if(!global_plan_.poses.empty()){
        global_plan_.poses.clear();
    }   
    global_plan_ = path;
    RCLCPP_INFO(logger_, "Received a new plan");
    
    auto msg = std::make_unique<nav_msgs::msg::Path>(global_plan_);
    global_plan_.header.stamp = path.header.stamp;
    global_plan_.header.frame_id = path.header.frame_id;  // or "odom"
    global_path_pub_->publish(std::move(msg));
  
    //RCLCPP_INFO(logger_, "global_plan_.orientation x y z w = [%lf] [%lf] [%lf] [%lf]", global_plan_.poses.back().pose.orientation.x, global_plan_.poses.back().pose.orientation.y, global_plan_.poses.back().pose.orientation.z, global_plan_.poses.back().pose.orientation.w);
    tf2::Quaternion q;
    tf2::fromMsg(global_plan_.poses.back().pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double pitch, row, yaw;
    qt.getRPY(pitch, row, yaw);
    //RCLCPP_INFO(logger_, "yaw is = [%lf]", yaw);
    final_goal_angle_ = yaw;

    update_plan_ = false;
    // update_plan_ = true;
    isObstacleExist_ = false;
    //print the distance between the points
    // RCLCPP_INFO(logger_, "global_plan_.poses.size() = [%d]", global_plan_.poses.size());
    // for(int i = 0; i < global_plan_.poses.size()-1; i++){
    //     RCLCPP_INFO(logger_, "distance between points [%d] and [%d] is [%lf]", i, i+1, euclidean_distance(global_plan_.poses[i].pose.position, global_plan_.poses[i+1].pose.position));
    // } 


    // for(int i = 0; i < latest_costmap_->data.size(); i++){
    //     RCLCPP_INFO(logger_, "costmap data is [%d]", latest_costmap_->data[i]);
    // }
    
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
    //RCLCPP_INFO(logger_, "vector_path final goal angle = [%lf]", vector_path[vector_path.size()-1].theta_);
    //RCLCPP_INFO(logger_, "vector_global_path_ final goal angle = [%lf]", vector_global_path_[vector_global_path_.size()-1].theta_);
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

//     // // for rviz visualization
//     // geometry_msgs::PoseStamped check_goal_;
//     // check_goal_.header.frame_id = frame_;
//     // check_goal_.header.stamp = ros::Time::now();
//     // check_goal_.pose.position.x = local_goal.x_;
//     // check_goal_.pose.position.y = local_goal.y_;

//     // tf2::Quaternion q;
//     // q.setRPY(0, 0, local_goal.theta_);
//     // check_goal_.pose.orientation.x = q.x();
//     // check_goal_.pose.orientation.y = q.y();
//     // check_goal_.pose.orientation.z = q.z();
//     // check_goal_.pose.orientation.w = q.w();
//     // local_goal_pub_.publish(check_goal_);
    
    //RCLCPP_INFO(logger_, "local_goal is [%lf] [%lf]", local_goal.x_, local_goal.y_);
    // RCLCPP_INFO(logger_, "angle = [%lf]", cur_pose.theta_);
    
    return local_goal;

    
}

double CustomController::getGoalAngle(double cur_angle, double goal_angle) {
    double ang_diff_ = goal_angle - cur_angle;
    double angular_max_vel_ = 2.0;
    double angle_vel_ = 0.0;
    double angular_kp_ = 4.0;
    if(cur_angle >= 0 && goal_angle >= 0){
            if(ang_diff_ >= 0) angle_vel_ = std::min((ang_diff_ * angular_kp_), angular_max_vel_);
            else angle_vel_ = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
        }
        else if(cur_angle < 0 && goal_angle < 0){
           if(ang_diff_ >= 0) angle_vel_ = std::min((ang_diff_ * angular_kp_), angular_max_vel_);
            else angle_vel_ = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
        }
        else if(cur_angle < 0 && goal_angle >= 0){
            if((fabs(cur_angle) + goal_angle) >= M_PI) angle_vel_ = std::max((-ang_diff_ * angular_kp_), -angular_max_vel_);
            else angle_vel_ = std::min((ang_diff_ * angular_kp_), angular_max_vel_); 
        }
        else{
            if((cur_angle + fabs(goal_angle)) <= M_PI) angle_vel_ = std::max((ang_diff_ * angular_kp_), -angular_max_vel_);
            else angle_vel_ = std::min((-ang_diff_ * angular_kp_), angular_max_vel_);
        }
    return angle_vel_;
    // if(goal_angle - cur_angle >= 3.1415926) {
    //     return (goal_angle - cur_angle - 3.1415926 * 2);
    // } else if(goal_angle - cur_angle < -3.1415926) {
    //     return (goal_angle - cur_angle + 3.1415926 * 2);
    // } else {
    //     return goal_angle;
    // }
}

int CustomController::getIndex(RobotState cur_pose, std::vector<RobotState> &path, double look_ahead_distance){
    if (path.empty()) {
        RCLCPP_INFO(logger_, "[%s] Path is empty", plugin_name_.c_str());
        return 0;
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
    return next_index;
}

bool CustomController::checkObstacle(int current_index, int check_index){
    
    if(current_index >= check_index){
        
        check_goal_.header.frame_id = global_plan_.header.frame_id;
        check_goal_.header.stamp = global_plan_.header.stamp;
        check_goal_.pose.position.x = vector_global_path_[current_index].x_;
        check_goal_.pose.position.y = vector_global_path_[current_index].y_;
        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>(check_goal_);
        tf2::Quaternion q;
        q.setRPY(0, 0, vector_global_path_[current_index].theta_);
        check_goal_.pose.orientation.x = q.x();
        check_goal_.pose.orientation.y = q.y();
        check_goal_.pose.orientation.z = q.z();
        check_goal_.pose.orientation.w = q.w();
        check_goal_pub_->publish(std::move(msg));
        for(int i = 0; i < current_index; i++){
            // check_goal_.header.frame_id = global_plan_.header.frame_id;
            // check_goal_.header.stamp = global_plan_.header.stamp;
            // check_goal_.pose.position.x = vector_global_path_[i].x_;
            // check_goal_.pose.position.y = vector_global_path_[i].y_;
            // auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>(check_goal_);
            // tf2::Quaternion q;
            // q.setRPY(0, 0, vector_global_path_[i].theta_);
            // check_goal_.pose.orientation.x = q.x();
            // check_goal_.pose.orientation.y = q.y();
            // check_goal_.pose.orientation.z = q.z();
            // check_goal_.pose.orientation.w = q.w();
            // check_goal_pub_->publish(std::move(msg));
            int mapX = vector_global_path_[i].x_ * 100;
            int mapY = vector_global_path_[i].y_ * 100;
            int index = (mapY-1) * 300 + mapX;
            if(latest_costmap_->data[index] > 60){
                //RCLCPP_INFO(logger_, "Obstacle data is [%d]", latest_costmap_->data[index]);
                return true;
                // return false;
            }
        }
        
    }else{
        
        check_goal_.header.frame_id = global_plan_.header.frame_id;
        check_goal_.header.stamp = global_plan_.header.stamp;
        check_goal_.pose.position.x = vector_global_path_[check_index].x_;
        check_goal_.pose.position.y = vector_global_path_[check_index].y_;
        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>(check_goal_);
        tf2::Quaternion q;
        q.setRPY(0, 0, vector_global_path_[check_index].theta_);
        check_goal_.pose.orientation.x = q.x();
        check_goal_.pose.orientation.y = q.y();
        check_goal_.pose.orientation.z = q.z();
        check_goal_.pose.orientation.w = q.w();
        check_goal_pub_->publish(std::move(msg));
        for(int i = current_index; i < check_index; i++){
            // check_goal_.header.frame_id = global_plan_.header.frame_id;
            // check_goal_.header.stamp = global_plan_.header.stamp;
            // check_goal_.pose.position.x = vector_global_path_[i].x_;
            // check_goal_.pose.position.y = vector_global_path_[i].y_;
            // auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>(check_goal_);
            // tf2::Quaternion q;
            // q.setRPY(0, 0, vector_global_path_[i].theta_);
            // check_goal_.pose.orientation.x = q.x();
            // check_goal_.pose.orientation.y = q.y();
            // check_goal_.pose.orientation.z = q.z();
            // check_goal_.pose.orientation.w = q.w();
            // check_goal_pub_->publish(std::move(msg));
            int mapX = vector_global_path_[i].x_ * 100;
            int mapY = vector_global_path_[i].y_ * 100;
            int index = (mapY-1) * 300 + mapX;
            if(latest_costmap_->data[index] > 60){
                RCLCPP_INFO(logger_, "Obstacle data is [%d]", latest_costmap_->data[index]);
                return true;
                // return false;
            }
        }
    }
    
    return false;
}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
    vector_global_path_.clear();
    posetoRobotState(pose.pose, cur_pose_);
    pathToVector(global_plan_, vector_global_path_);
    // cmd_vel
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    if(!goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)){
        rival_distance_ = sqrt(pow(rival_pose_.pose.pose.position.x - cur_pose_.x_, 2) + pow(rival_pose_.pose.pose.position.y - cur_pose_.y_, 2));
        //RCLCPP_INFO(logger_, "Rival distance is [%lf]", rival_distance_);
        
        
        local_goal_ = getLookAheadPoint(cur_pose_, vector_global_path_, look_ahead_distance_);
        
        check_index_ = 0;
        current_index_ = 0;
        last_vel_x_ = cmd_vel.twist.linear.x;
        last_vel_y_ = cmd_vel.twist.linear.y;
        //RCLCPP_INFO(logger_, "last_vel = []")
        //RCLCPP_INFO(logger_, "vector_global_path_ final goal angle after get LAD is = [%lf]", vector_global_path_[vector_global_path_.size()-1].theta_);
        //final_goal_angle_ = vector_global_path_[vector_global_path_.size()-1].theta_;
        //RCLCPP_INFO(logger_, "final goal angle is [%lf]", final_goal_angle_); 
        double global_distance = sqrt(pow(global_plan_.poses.back().pose.position.x - cur_pose_.x_, 2) + pow(global_plan_.poses.back().pose.position.y - cur_pose_.y_, 2));
        local_goal_ = globalTOlocal(cur_pose_, local_goal_);
        double local_angle = atan2(local_goal_.y_, local_goal_.x_);
        // posetoRobotState(rival_pose_.pose, local_rival_pose_);
        // local_rival_pose_ = globalTOlocal(cur_pose_, local_rival_pose_);

        // //rival_to_move_angle = atan2(rival_pose_.pose.pose.position.y - cur_pose_.y_, rival_pose_.pose.pose.position.x - cur_pose_.x_);
        // RCLCPP_INFO(logger_, "rival to move angle = [%lf]", rival_to_move_angle);
        if(rival_distance_ < 1){
            //RCLCPP_INFO(logger_, "Rival is too close");
            max_linear_vel_ = 0.2;
        }else{
            max_linear_vel_ = 0.4;
        }
        double local_distance = sqrt(pow(local_goal_.x_ - cur_pose_.x_, 2) + pow(local_goal_.y_ - cur_pose_.y_, 2));
        //RCLCPP_INFO(logger_, "final goal angle is [%lf]", vector_global_path_[vector_global_path_.size()-1].theta_);
        //RCLCPP_INFO(logger_, "cur_pose angle is [%lf]", cur_pose_.theta_);
        cmd_vel.twist.linear.x = std::min(global_distance * 1.5, max_linear_vel_) * cos(local_angle);
        cmd_vel.twist.linear.y = std::min(global_distance * 1.5, max_linear_vel_) * sin(local_angle);
        cmd_vel.twist.angular.z = getGoalAngle(cur_pose_.theta_, final_goal_angle_);
        double vel_ = sqrt(pow(cmd_vel.twist.linear.x, 2) + pow(cmd_vel.twist.linear.y, 2));
        check_distance_ = std::max(vel_ * 2.5,look_ahead_distance_);
        //RCLCPP_INFO(logger_, "check_distance is [%lf]", check_distance_);
        check_index_ = getIndex(cur_pose_, vector_global_path_, check_distance_);
        current_index_ = getIndex(cur_pose_, vector_global_path_, look_ahead_distance_);
        // RCLCPP_INFO(logger_, "check_index is [%d]", check_index_);        
        // RCLCPP_INFO(logger_, "current_index is [%d]", current_index_);
        // RCLCPP_INFO(logger_, "vector_global_path size is [%d]", vector_global_path_.size());
        // RCLCPP_INFO(logger_, "global_path size is [%d]", global_plan_.poses.size());
        //RCLCPP_INFO(logger_, "final_goal_angle is [%lf]", final_goal_angle_);
        //RCLCPP_INFO(logger_, "cmd_vel is [%lf] [%lf] [%lf]", cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
        //RCLCPP_INFO(logger_, "local_angle is [%lf]", local_angle);
        isObstacleExist_ = checkObstacle(current_index_, check_index_);
        if(isObstacleExist_){
            cmd_vel.twist.linear.x = last_vel_x_;
            cmd_vel.twist.linear.y = last_vel_y_;
            cmd_vel.twist.angular.z = 0.0;
            update_plan_ = true;
            return cmd_vel;
        }
        
        return cmd_vel;
    }
    else if(fabs(final_goal_angle_ - cur_pose_.theta_) > 0.01){
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = getGoalAngle(cur_pose_.theta_, final_goal_angle_);
        return cmd_vel;
    }
    else{
        RCLCPP_INFO(logger_, "[%s] Goal reached", plugin_name_.c_str());
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        update_plan_ = true;
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