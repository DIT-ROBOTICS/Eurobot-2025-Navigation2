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
                        target_point.x = 0;
                        target_point.y = 0;
                        target_point.z = 0;
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

    Status Escape::onRun(const std::shared_ptr<const Escape::Goal> command){
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

    void Escape::costmapCallback(const nav_msgs::msg::OccupancyGrid& msg){
        RCLCPP_INFO(logger_, "In call back");
        costmap = msg;
        RCLCPP_INFO(logger_, "Costmap is available., %d x %d", msg.info.width, msg.info.height);
    }

    void Escape::rivalCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg){
        rivalPose = msg;
        RCLCPP_INFO(logger_, "Rival pose is available.");
        RCLCPP_INFO(logger_, "Rival pose is %f, %f", rivalPose.pose.pose.position.x, rivalPose.pose.pose.position.y);
    }

    bool Escape::scanRadius() {
        // Define scan parameters
        const double scan_radius = 0.5;  // meters
        const int num_points = 36;  // number of points to check on each circle
        const double angle_increment = 2 * M_PI / num_points;
        
        // Get current robot pose
        if (!nav2_util::getCurrentPose(robotPose, *tf_buffer_)) {
            RCLCPP_ERROR(logger_, "Failed to get current pose");
            return false;
        }
        
        // Check circles with increasing radius
        for (double r = 0.1; r <= scan_radius; r += 0.1) {
            // Check points around the circle
            for (double angle = 0; angle < 2 * M_PI; angle += angle_increment) {
                // Calculate point coordinates
                double world_x = robotPose.pose.position.x + r * cos(angle);
                double world_y = robotPose.pose.position.y + r * sin(angle);
                
                // Convert world coordinates to map coordinates
                double map_x = (world_x - costmap.info.origin.position.x) / costmap.info.resolution;
                double map_y = (world_y - costmap.info.origin.position.y) / costmap.info.resolution;
                
                // Check if point is within map bounds
                if (map_x >= 0 && map_x < costmap.info.width && 
                    map_y >= 0 && map_y < costmap.info.height) {
                        
                    double cost = getOneGridCost(map_x, map_y);
                    // If cost is above threshold (obstacle), return false
                    if (cost > 50) {  // Assuming cost threshold of 50
                        RCLCPP_INFO(logger_, "Obstacle detected at radius %f, angle %f", r, angle);
                        return false;
                    }
                }
            }
        }
        
        // If we get here, all circles were clear
        RCLCPP_INFO(logger_, "Path is clear for straight motion");
        return true;
    }

    geometry_msgs::msg::Point Escape::findTargetPoint() {
        // Get current robot pose
        if (!nav2_util::getCurrentPose(robotPose, *tf_buffer_)) {
            RCLCPP_ERROR(logger_, "Failed to get current pose");
            return robotPose.pose.position;
        }

        const double scan_radius = 0.5;  // meters
        const int num_points = 36;  // points per circle
        const double angle_increment = 2 * M_PI / num_points;
        
        double lowest_cost = 100.0;  // Max cost threshold
        geometry_msgs::msg::Point best_point = robotPose.pose.position;
        
        // Scan increasing radius circles
        for (double r = 0.1; r <= scan_radius; r += 0.1) {
            for (double angle = 0; angle < 2 * M_PI; angle += angle_increment) {
                // Calculate world coordinates
                double world_x = robotPose.pose.position.x + r * cos(angle);
                double world_y = robotPose.pose.position.y + r * sin(angle);
                
                // Convert to map coordinates
                double map_x = (world_x - costmap.info.origin.position.x) / costmap.info.resolution;
                double map_y = (world_y - costmap.info.origin.position.y) / costmap.info.resolution;
                
                // Check map bounds
                if (map_x >= 0 && map_x < costmap.info.width && 
                    map_y >= 0 && map_y < costmap.info.height) {
                    
                    double cost = getOneGridCost(map_x, map_y);
                    if (cost < lowest_cost) {
                        lowest_cost = cost;
                        best_point.x = world_x;
                        best_point.y = world_y;
                        best_point.z = 0.0;
                        
                        // If we find a very good point, return immediately
                        if (cost < 10) {
                            RCLCPP_INFO(logger_, "Found good target point at (%f, %f) with cost %f",
                                      world_x, world_y, cost);
                            return best_point;
                        }
                    }
                }
            }
        }
        
        if (lowest_cost < 100.0) {
            RCLCPP_INFO(logger_, "Found target point at (%f, %f) with cost %f",
                        best_point.x, best_point.y, lowest_cost);
        } else {
            RCLCPP_WARN(logger_, "No suitable target point found, staying in place");
        }
        
        return best_point;
    }

    double Escape::getOneGridCost(double map_x, double map_y){
        int index_cost = (int)((map_y-1)*300+map_x);
        return costmap.data[index_cost];
    }

    geomety_msgs::msg::Twist makeMove(double x, double y){
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        double vel_x, vel_y, max_vel;
        double dist = hypot(robotPose.pose.position.x - x, robotPose.pose.position.y - y);
        double ang_diff;
        double first_ang_diff = atan2(y - robotPose.pose.position.y, x - robotPose.pose.position.x);
        double cur_linear_kp = 2;
        double linear_max_vel;
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

   
    Status Escape::onCycleUpdate(){
        rclcpp::Duration time_remaining = end_time - this->clock_->now();
        if(scanRadius()){
            stopRobot();
            RCLCPP_INFO(logger_, "Escape successfully");
            return Status::SUCCEEDED;
        }
        if(!nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)){
            RCLCPP_ERROR(logger_, "Current robot pose is not available.");
            return Status::FAILED;
        }

        target_point = findTargetPoint();
        if(target_point.x == robotPose.pose.position.x){
            RCLCPP_INFO(logger_, "No target point found");
            return Status::FAILED;
        }

        // const double current_yaw = tf2::getYaw(robotPose.pose.position);
        
        auto cmd_vel = makeMove(target_point.x, target_point.y);
        vel_pub_->publish(std::move(cmd_vel));
        return Status::RUNNING;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Escape, nav2_core::Behavior)