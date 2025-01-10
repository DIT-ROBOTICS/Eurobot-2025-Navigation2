#include "nav2_behaviors/plugins/run.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{
    Run::Run() : TimedBehavior<RunAction>(),
                    feedback(std::make_shared<RunAction::Feedback>()),
                    min_linear_vel(0.0),
                    max_linear_vel(0.0),
                    cmd_yaw(0.0),
                    prev_yaw(0.0),
                    relative_yaw(0.0)
                    {
                        auto node_test = node_.lock();
                        rclNode = std::make_shared<rclcpp::Node>("costmap_sub_node");
                        subscription = rclNode->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", 10, std::bind(&Run::costmapCallback, this, std::placeholders::_1));
                    }
    Run::~Run() = default;

    void Run::onConfigure(){
        min_linear_vel = 0.0;
        max_linear_vel = 1.0;
    }

    Status Run::onRun(const std::shared_ptr<const RunAction::Goal> command){
        geometry_msgs::msg::PoseStamped current_pose;
        if(!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)){
            RCLCPP_ERROR(logger_, "Current robot pose is not available.");
            return Status::FAILED;
        }

        // nav2_msgs::msg::Costmap costmap;
        // if(!nav2_util::Costmap::get_costmap(costmap)){
        //     RCLCPP_ERROR(logger_, "Costmap is not available.");
        //     return Status::FAILED;
        // }
        // RCLCPP_INFO(logger_, "Costmap is available.");


        // prev_yaw = tf2::getYaw(current_pose.pose.orientation);
        relative_yaw = 0.0;

        cmd_yaw = command->target_yaw;

        RCLCPP_INFO(logger_, "Running %0.2f for run behavior.", cmd_yaw);
        //test
        command_time_allowance = command->time_allowance;
        end_time = this->clock_->now() + command_time_allowance;
        
        return Status::SUCCEEDED;
    }

    void Run::costmapCallback(const nav_msgs::msg::OccupancyGrid& msg){
        RCLCPP_INFO(logger_, "In call back");
        costmap = msg;
        RCLCPP_INFO(logger_, "Costmap is available., %d x %d", msg.info.width, msg.info.height);
    }

    Status Run::onCycleUpdate(){
        RCLCPP_INFO(logger_, "the map is %d x %d", costmap.info.width, costmap.info.height);
        rclcpp::Duration time_remaining = end_time - this->clock_->now();
        if(time_remaining.seconds() < 0.0 && command_time_allowance.seconds() > 0.0){
            stopRobot();
            RCLCPP_WARN(logger_, "Exceeded time allowance before reaching the Run goal - Exiting Run");
            return Status::FAILED;
        }

        geometry_msgs::msg::PoseStamped current_pose;
        if(!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)){
            RCLCPP_ERROR(logger_, "Current robot pose is not available.");
            return Status::FAILED;
        }

        // const double current_yaw = tf2::getYaw(current_pose.pose.position);
        const double current_yaw = 0;
        RCLCPP_INFO(logger_ ,"Current yaw: %f", current_yaw);              
        double vel = 1;
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel->linear.x = vel;
        vel_pub_->publish(std::move(cmd_vel));
        return Status::RUNNING;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Run, nav2_core::Behavior)