#ifndef CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace custom_controller{

class CustomController : public nav2_core::Controller{
    public:
        CustomController() = default;
        ~CustomController() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;
        // void setSpeedLimit(double speed_limit, double speed_limit_yaw) override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped & pose,
            const geometry_msgs::msg::Twist & velocity,
            nav2_core::GoalChecker * goal_checker) override;

        void setPlan(const nav_msgs::msg::Path & path) override;

    protected:
        // Setup parameters
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Logger logger_ {rclcpp::get_logger("CustomController")};
        rclcpp::Clock::SharedPtr clock_;

        // Parameters from the config file
        double max_linear_vel_, min_linear_vel_;
        double max_angular_vel_, min_angular_vel_;
        double max_linear_acc_, max_angular_acc_;
        double yaw_goal_tolerance_;
        double angular_kp_;
        rclcpp::Duration transform_tolerance_ {0, 0};

        // Variables
        int yaw_debounce_counter_ = 0;

        nav_msgs::msg::Path global_plan_;  // Store the global plan
};

}  // namespace custom_controller

#endif  // CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_ aka path_executor