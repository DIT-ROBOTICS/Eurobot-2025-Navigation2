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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

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
        bool isRunAway(
            const double & distance,
            geometry_msgs::msg::Twist * cmd_vel,
            geometry_msgs::msg::Pose2D & pose2d);
        double map_x, map_y;
        double rival_x, rival_y;
        int scan_radius;
        double getOneGridCost(double map_x, double map_y);
        double** scanSquard;
        void findscanSquardCost(double center_x, double center_y);
        void costmapCallback(const nav_msgs::msg::OccupancyGrid& msg);
        void rivalCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
        geometry_msgs::msg::PoseStamped robotPose;
        geometry_msgs::msg::PoseWithCovarianceStamped rivalPose;


        EscapeAction::Feedback::SharedPtr feedback;
        double min_linear_vel;
        double max_linear_vel;
        double cmd_yaw;
        double prev_yaw;
        double relative_yaw;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_rival;
        nav_msgs::msg::OccupancyGrid costmap;
        rclcpp::Duration command_time_allowance{0,0};
        rclcpp::Time end_time;
    };
}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__ESCAPE_HPP_