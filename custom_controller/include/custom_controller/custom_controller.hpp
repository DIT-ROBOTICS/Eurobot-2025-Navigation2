#ifndef CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

namespace custom_controller{
class RobotState {
   public:
    RobotState(double x, double y, double theta);
    RobotState() {}
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
        void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
        // void setSpeedLimit(double speed_limit, double speed_limit_yaw) override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped & pose,
            const geometry_msgs::msg::Twist & velocity,
            nav2_core::GoalChecker * goal_checker) override;

        void setPlan(const nav_msgs::msg::Path & path) override;
        void pathToVector(nav_msgs::msg::Path path, std::vector<RobotState> &vector_path);
        void posetoRobotState(geometry_msgs::msg::Pose pose, RobotState &state);
        double getGoalAngle(double cur_pose, double goal);
        RobotState getLookAheadPoint(RobotState cur_pose, std::vector<RobotState> &path, double look_ahead_distance);
        RobotState globalTOlocal(RobotState cur_pose, RobotState goal);
        int getIndex(RobotState cur_pose, std::vector<RobotState> &path, double look_ahead_distance);
        bool checkObstacle(int current_index, int check_index);
    protected:
        // Setup parameters
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        
        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rival_pose_subscription_;
        nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_; // Store the received costmap
        nav_msgs::msg::Odometry rival_pose_; // Store the received rival pose
        rclcpp::Logger logger_{rclcpp::get_logger("CustomController")};

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rival_distance_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reach_pub_;
        
        rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

        // Parameters from the config file
        double control_frequency_;
        double max_linear_vel_, min_linear_vel_;
        double linear_acceleration_;
        double max_angular_vel_, min_angular_vel_;
        double max_linear_acc_, max_angular_acc_;
        double yaw_goal_tolerance_;
        double angular_kp_;
        double linear_kp_;
        double look_ahead_distance_;
        double speed_test;
        double final_goal_angle_;
        double rival_distance_;
        double rival_to_move_angle;
        double last_vel_x_;
        double last_vel_y_;

        double speed_decade_;
        int costmap_tolerance_;

        double target_vel_x_;
        double target_vel_y_;
  
        rclcpp::Duration transform_tolerance_ {0, 0};

        // Variables
        int yaw_debounce_counter_ = 0;

        nav_msgs::msg::Path global_plan_;  // Store the global plan
        geometry_msgs::msg::PoseStamped check_goal_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> check_goal_pub_;
        std::vector<RobotState> vector_global_path_;
        RobotState goal_pose_;
        RobotState local_goal_;
        RobotState cur_pose_;
        RobotState velocity_state_;
        RobotState local_rival_pose_;
        RobotState cur_goal_pose_;
        bool update_plan_;
        double check_distance_;
        int check_index_;
        int current_index_;
        bool isObstacleExist_;
        bool keep_planning_;

        // Special function for controller
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_function_sub_;
        std::string controller_function_;
        double spin_delay_threshold_;
};

}  // namespace custom_controller

#endif  // CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_ aka path_executor