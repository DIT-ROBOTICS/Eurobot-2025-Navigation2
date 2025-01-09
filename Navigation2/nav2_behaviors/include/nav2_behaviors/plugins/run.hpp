#ifndef NAV2_BEHAVIORS__PLUGINS__RUN_HPP_
#define NAV2_BEHAVIORS__PLUGINS__RUN_HPP_

#include "nav2_msgs/action/run.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


namespace nav2_behaviors
{
    using RunAction = nav2_msgs::action::Run;

    class Run: public TimedBehavior<RunAction>
    {
    public:
        Run();
        ~Run();
        Status onRun(const std::shared_ptr<const RunAction::Goal> command) override;
        void onConfigure() override;
        Status onCycleUpdate() override;

    protected:
        bool isRunAway(
            const double & distance,
            geometry_msgs::msg::Twist * cmd_vel,
            geometry_msgs::msg::Pose2D & pose2d);
        void costmapCallback(const nav_msgs::msg::OccupancyGrid& msg);

        RunAction::Feedback::SharedPtr feedback;
        double min_linear_vel;
        double max_linear_vel;
        double cmd_yaw;
        double prev_yaw;
        double relative_yaw;
        std::shared_ptr<rclcpp::Node> rclNode;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription;
        nav_msgs::msg::OccupancyGrid costmap;
        rclcpp::Duration command_time_allowance{0,0};
        rclcpp::Time end_time;
    };
}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__RUN_HPP_