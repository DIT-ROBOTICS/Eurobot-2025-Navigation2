#ifndef NAV2_BEHAVIORS__PLUGINS__SHRINK_HPP_
#define NAV2_BEHAVIORS__PLUGINS__SHRINK_HPP_

#include "nav2_msgs/action/shrink.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

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
        bool noCostInMiddle();
        bool noCostAtGoal();
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shrinkback_pub;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr shrinkCheck_srv;
        void handleShrinkCheck(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setMode_rival_client;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setMode_object_client;

    protected:
        int times;
        int unused_shrink;
        bool shrinkBack;
        double costmap_tolerance;
        double original_inflation_radius;
        double original_rival_halted_radius;
        double original_rival_wandering_radius;
        double original_rival_moving_radius;
        double original_rival_unknown_radius;
        double original_object_board_radius;
        double original_object_column_radius;
        double getOneGridCost(double x, double y);
        void costmapCallback(const nav_msgs::msg::OccupancyGrid& msg);
        void goalPoseCallback(const geometry_msgs::msg::PoseStamped& msg);
        void worldToMap(double wx, double wy, int & mx, int & my);
        geometry_msgs::msg::PoseStamped robotPose;
        geometry_msgs::msg::PoseStamped goalPose;
        nav_msgs::msg::OccupancyGrid costmap;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
        rclcpp::AsyncParametersClient::SharedPtr radius_param_client;
        rclcpp::Duration command_time_allowance{0,0};
        rclcpp::Time end_time;
        void changeInflationLayer(bool doShrink);
        void changeRivalLayer(bool doShrink);
        void changeObjectLayer(bool doShrink);
        void getOriginalParam();
        void setToOriginal();
        void setToShrink();
        void tellStopToShrinkBack();
    };
}

#endif // NAV2_BEHAVIORS__PLUGINS__SHRINK_HPP_