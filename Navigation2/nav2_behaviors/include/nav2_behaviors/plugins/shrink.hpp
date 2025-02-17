#ifndef NAV2_BEHAVIORS__PLUGINS__SHRINK_HPP_
#define NAV2_BEHAVIORS__PLUGINS__SHRINK_HPP_

#include "nav2_msgs/action/shrink.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behaviors
{
    using ShrinkAction = nav2_msgs::action::Shrink;

    class Shrink : public TimedBehavior<ShrinkAction>
    {
    public:
        Shrink();
        ~Shrink();
        Status onRun(const std::shared_ptr<const ShrinkAction::Goal> command) override;
        void onConfigure() override;
        Status onCycleUpdate() override;
        bool isWork();

    protected:
        int times;
        double inflation_radius;
        double getOneGridCost(double x, double y);
        void costmapCallback(const nav_msgs::msg::OccupancyGrid& msg);
        void worldToMap(double wx, double wy, int & mx, int & my);
        void setInflationRadius(double radius);
        geometry_msgs::msg::PoseStamped robotPose;
        nav_msgs::msg::OccupancyGrid costmap;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap;
        rclcpp::AsyncParametersClient::SharedPtr param_client;
        rclcpp::Duration command_time_allowance{0,0};
        rclcpp::Time end_time;
    };
}

#endif // NAV2_BEHAVIORS__PLUGINS__SHRINK_HPP_