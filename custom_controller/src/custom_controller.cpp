#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "custom_controller/custom_controller.hpp"
#include "nav2_util/geometry_utils.hpp"

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

    // Get parameters from the config file
    node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
    node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
    node->get_parameter(plugin_name_ + ".min_angular_vel", min_angular_vel_);
    node->get_parameter(plugin_name_ + ".max_linear_acc", max_linear_acc_);
    node->get_parameter(plugin_name_ + ".max_angular_acc", max_angular_acc_);
    node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
    node->get_parameter(plugin_name_ + ".angular_kp", angular_kp_);
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
    global_path_ = path;
}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
    // cmd_vel
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    if(!goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)){
        // Spin
        double dyaw = angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation), tf2::getYaw(global_plan_.poses.back().pose.orientation));
        if (fabs(dyaw) > yaw_goal_tolerance_) {
            yaw_debounce_counter_ = 0;
        } else {
            yaw_debounce_counter_++;
        }

        if(yaw_debounce_counter_ > 5){
            RCLCPP_INFO(logger_, "[%s] Orientation reached", plugin_name_.c_str());
            cmd_vel.twist.angular.z = 0.0;
        } else {
            cmd_vel.twist.angular.z = 1.0;
        }

        // Move
        // cmd_vel.twist.linear.x = 0.1;
        // cmd_vel.twist.linear.y = 0.1;
            
    } else {
        RCLCPP_INFO(logger_, "[%s] Goal reached", plugin_name_.c_str());
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;
    }

    return cmd_vel;
}

}  // namespace custom_controller

PLUGINLIB_EXPORT_CLASS(custom_controller::CustomController, nav2_core::Controller)