#include "nav2_behaviors/plugins/run.hpp"

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
                        scan_radius = 20;
                        map_x = 0;
                        map_y = 0;
                        scanSquard = new double*[scan_radius];
                        for(int i = 0; i < scan_radius; i++) {
                            scanSquard[i] = new double[scan_radius];
                        }
                    }
    Run::~Run() {
        for(int i = 0; i < scan_radius; i++) {
            delete[] scanSquard[i];
        }
        delete[] scanSquard;
    }

    void Run::onConfigure(){
        min_linear_vel = 0.0;
        max_linear_vel = 1.0;
        auto node_test = node_.lock();
        sub_costmap = node_test->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", rclcpp::QoS(10), std::bind(&Run::costmapCallback, this, std::placeholders::_1));
        sub_rival = node_test->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/rival_pose", rclcpp::QoS(10), std::bind(&Run::rivalCallback, this, std::placeholders::_1));
    }

    Status Run::onRun(const std::shared_ptr<const RunAction::Goal> command){
        if(!nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)){
            RCLCPP_ERROR(logger_, "Current robot pose is not available.");
            return Status::FAILED;
        }
        map_x = robotPose.pose.position.x * 100;
        map_y = robotPose.pose.position.y * 100;
        rival_x = rivalPose.pose.pose.position.x * 100;
        rival_y = rivalPose.pose.pose.position.y * 100;

        relative_yaw = 0.0;

        cmd_yaw = command->target_yaw;

        RCLCPP_INFO(logger_, "Running %0.2f for run behavior.", cmd_yaw);
        //testing
        command_time_allowance = command->time_allowance;
        end_time = this->clock_->now() + command_time_allowance;
        
        return Status::SUCCEEDED;
    }

    void Run::costmapCallback(const nav_msgs::msg::OccupancyGrid& msg){
        RCLCPP_INFO(logger_, "In call back");
        costmap = msg;
        RCLCPP_INFO(logger_, "Costmap is available., %d x %d", msg.info.width, msg.info.height);
    }

    void Run::rivalCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg){
        rivalPose = msg;
        RCLCPP_INFO(logger_, "Rival pose is available.");
        RCLCPP_INFO(logger_, "Rival pose is %f, %f", rivalPose.pose.pose.position.x, rivalPose.pose.pose.position.y);
    }

    double Run::getOneGridCost(double map_x, double map_y){
        int index_cost = (int)((map_y-1)*300+map_x);
        return costmap.data[index_cost];
    }

    void Run::findscanSquardCost(double center_x, double center_y){
        for(int i = 0;i<scan_radius;i++){
        for(int j = 0;j<scan_radius;j++){
            int index_cost = (center_y - scan_radius/2 + i -1) * 300 + center_x - scan_radius/2 + j;
            scanSquard[i][j] = costmap.data[index_cost];
        }
    }
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