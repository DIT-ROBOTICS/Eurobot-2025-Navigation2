#include "nav2_behaviors/plugins/escape.hpp"

namespace nav2_behaviors
{
    Escape::Escape() : TimedBehavior<EscapeAction>(),
                    feedback(std::make_shared<EscapeAction::Feedback>()),
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
                        target_point.position.x = 0;
                        target_point.position.y = 0;
                        target_point.position.z = 0;
                    }
    Escape::~Escape() {
        for(int i = 0; i < scan_radius; i++) {
            delete[] scanSquard[i];
        }
        delete[] scanSquard;
    }

    void Escape::onConfigure(){
        min_linear_vel = 0.0;
        max_linear_vel = 1.0;
        auto node_test = node_.lock();
        sub_costmap = node_test->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", rclcpp::QoS(10), std::bind(&Escape::costmapCallback, this, std::placeholders::_1));
        sub_rival = node_test->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/rival_pose", rclcpp::QoS(10), std::bind(&Escape::rivalCallback, this, std::placeholders::_1));
    }

    Status Escape::onRun(const std::shared_ptr<const EscapeAction::Goal> command){
        if(!nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)){
            return Status::FAILED;
        }
        map_x = robotPose.pose.position.x * 100;
        map_y = robotPose.pose.position.y * 100;
        rival_x = rivalPose.pose.pose.position.x * 100;
        rival_y = rivalPose.pose.pose.position.y * 100;
        map_width = costmap.info.width;
        map_height = costmap.info.height;

        //testing
        command_time_allowance = command->time_allowance;
        end_time = this->clock_->now() + command_time_allowance;
        if(isEscape()){
            stopRobot();
            RCLCPP_INFO(logger_, "Escape successfully");
            return Status::SUCCEEDED;
        }
        return Status::SUCCEEDED;
    }

    void Escape::costmapCallback(const nav_msgs::msg::OccupancyGrid& msg){
        costmap = msg;
    }

    void Escape::rivalCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg){
        rivalPose = msg;
    }

    bool Escape::isEscape() {
        // If we get here, all circles were clear
        int cost = getOneGridCost(robotPose.pose.position.x, robotPose.pose.position.y);
        if(cost > 50){
            return false;
        }
        else {
            return true;
        }
    }

    geometry_msgs::msg::Pose Escape::findTargetPoint() {
        const double scan_radius = 0.3;  // meters
        const int num_points = 36;  // points per circle
        const double angle_increment = 2 * M_PI / num_points;
        
        double lowest_cost = 100.0;  // Max cost threshold
        geometry_msgs::msg::Pose best_point = robotPose.pose;
        double robot_cost = getOneGridCost(robotPose.pose.position.x, robotPose.pose.position.y);
        
        // Scan increasing radius circles
        for (double r = 0.01; r <= scan_radius; r += 0.01) {
            for (double angle = 0; angle < 2 * M_PI; angle += angle_increment) {
                // Calculate world coordinates
                double world_x = robotPose.pose.position.x + r * cos(angle);
                double world_y = robotPose.pose.position.y + r * sin(angle);
                
                // Convert to map coordinates
                int map_x, map_y;
                worldToMap(world_x, world_y, map_x, map_y);
                // Check map bounds
                if (map_x >= 0 && map_x < map_width && 
                    map_y >= 0 && map_y < map_height) {
                    
                    double cost = getOneGridCost(world_x, world_y);
                    if (cost < lowest_cost) {
                        lowest_cost = cost;
                        best_point.position.x = world_x;
                        best_point.position.y = world_y;
                        best_point.position.z = 0.0;
                        
                        // If we find a very good point, return immediately
                        if (cost == 0) {
                            return best_point;
                        }
                    }
                }
            }
            if(lowest_cost < robot_cost){
                return best_point;
            }
        }
        
        if (lowest_cost < 100.0) {
            RCLCPP_INFO(logger_, "Found target point at (%f, %f) with cost %f",
                        best_point.position.x, best_point.position.y, lowest_cost);
        } else {
            RCLCPP_WARN(logger_, "No suitable target point found, staying in place");
        }
        
        return best_point;
    }

    double Escape::getOneGridCost(double x, double y){
        int map_x, map_y;
        worldToMap(x, y, map_x, map_y);
        int index_cost = (int)((map_y)*300+map_x);
        return costmap.data[index_cost];
    }

    std::unique_ptr<geometry_msgs::msg::Twist> Escape::makeMove(double x, double y){
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        double vel_x, vel_y, max_vel;
        double dist = hypot(this->robotPose.pose.position.x - x, this->robotPose.pose.position.y - y);
        // double ang_diff;
        double first_ang_diff = atan2(y - this->robotPose.pose.position.y, x - this->robotPose.pose.position.x);
        double cur_linear_kp = 5;
        double linear_max_vel = 0.4;
        max_vel = std::min(dist*cur_linear_kp, linear_max_vel);
        vel_x = max_vel * cos(first_ang_diff);
        vel_y = max_vel * sin(first_ang_diff);
        cmd_vel->linear.x = vel_x;
        cmd_vel->linear.y = vel_y;
        cmd_vel->linear.z = 0;
        cmd_vel->angular.x = 0;
        cmd_vel->angular.y = 0;
        cmd_vel->angular.z = 0;
        return cmd_vel;
    }

    bool Escape::outOfBound(double x, double y){
        int map_x, map_y;
        worldToMap(x, y, map_x, map_y);
        if(map_x < 0 || map_x > map_width || map_y < 0 || map_y > map_height){
            return true;
        }
        return false;
    }

    void Escape::worldToMap(double wx, double wy, int & mx, int & my){
        mx = (int)((wx - costmap.info.origin.position.x) / costmap.info.resolution);
        my = (int)((wy - costmap.info.origin.position.y) / costmap.info.resolution);
    }

   
    Status Escape::onCycleUpdate(){
        rclcpp::Duration time_remaining = end_time - this->clock_->now();
        if(!nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)){
            stopRobot();
            return Status::FAILED;
        }
        else {
            // output the current robot pose in original pose value
        }
        if(isEscape() || outOfBound(robotPose.pose.position.x, robotPose.pose.position.y)){
            stopRobot();
            RCLCPP_INFO(logger_, "Escape successfully");
            return Status::SUCCEEDED;
        }

        target_point = findTargetPoint();
        if(target_point.position.x == robotPose.pose.position.x && target_point.position.y == robotPose.pose.position.y){
            stopRobot();
            return Status::FAILED;
        }

        // const double current_yaw = tf2::getYaw(robotPose.pose.position);
        
        auto cmd_vel = makeMove(target_point.position.x, target_point.position.y);
        vel_pub_->publish(std::move(cmd_vel));
        return Status::RUNNING;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Escape, nav2_core::Behavior)