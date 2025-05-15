#ifndef NAV2_BEHAVIORS__PLUGINS__ESCAPE_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ESCAPE_HPP_

#include "nav2_msgs/action/escape.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_behaviors
{
    using EscapeAction = nav2_msgs::action::Escape;

    class Escape: public TimedBehavior<EscapeAction>
    {
    public:
        Escape();
        ~Escape();
        Status onRun(const std::shared_ptr<const EscapeAction::Goal> command) override;
        void onConfigure() override;
        Status onCycleUpdate() override;

    protected:
        rclcpp::TimerBase::SharedPtr target_update_timer_;
        void targetUpdateCallback();

        double map_x, map_y;
        int map_width, map_height;
        double rival_x, rival_y;
        int scan_radius;
        double getOneGridCost(double x, double y);
        bool isEscape();
        bool outOfBound(double x, double map_y);
        void worldToMap(double wx, double wy, int & mx, int & my);
        geometry_msgs::msg::Pose target_point;
        geometry_msgs::msg::Pose findTargetPoint();
        std::unique_ptr<geometry_msgs::msg::Twist> makeMove(double x, double y);
        void costmapCallback(const nav_msgs::msg::OccupancyGrid& msg);
        void rivalCallback(const nav_msgs::msg::Odometry& msg);
        geometry_msgs::msg::PoseStamped robotPose;
        nav_msgs::msg::Odometry rivalPose;

        EscapeAction::Feedback::SharedPtr feedback;
        double min_linear_vel;
        double max_linear_vel;
        double cmd_yaw;
        double prev_yaw;
        double relative_yaw;
        double target_update_frequency_;
        bool is_active = false;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_rival;
        nav_msgs::msg::OccupancyGrid costmap;
    };
}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__ESCAPE_HPP_