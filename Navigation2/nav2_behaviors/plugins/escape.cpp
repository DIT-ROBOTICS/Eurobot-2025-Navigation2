#include "nav2_behaviors/plugins/escape.hpp"

namespace nav2_behaviors
{
    Escape::Escape() : TimedBehavior<EscapeAction>(),
                    feedback(std::make_shared<EscapeAction::Feedback>()),
                    min_linear_vel(0.0),
                    max_linear_vel(0.0),
                    cmd_yaw(0.0),
                    prev_yaw(0.0),
                    relative_yaw(0.0),
                    target_update_frequency_(5.0),  // Default to 1Hz updates
                    is_active(false)
                    {
                        scan_radius = 20;
                        map_x = 0;
                        map_y = 0;
                        target_point.position.x = 0;
                        target_point.position.y = 0;
                        target_point.position.z = 0;
                    }

    Escape::~Escape() {
        // Make sure to stop the timer if it's running
        if (target_update_timer_) {
            target_update_timer_->cancel();
        }
    }

    void Escape::onConfigure() {
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error("Failed to lock node in onConfigure");
        }
        
        // Create the timer for updating target points
        target_update_timer_ = node->create_wall_timer(
            std::chrono::duration<double>(1.0 / target_update_frequency_),
            std::bind(&Escape::targetUpdateCallback, this));
            
        // Existing configuration
        min_linear_vel = 0.0;
        max_linear_vel = 1.0;
        
        sub_costmap = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 
            rclcpp::QoS(10), 
            std::bind(&Escape::costmapCallback, this, std::placeholders::_1));
            
        sub_rival = node->create_subscription<nav_msgs::msg::Odometry>(
            "/rival_pose", 
            rclcpp::QoS(10), 
            std::bind(&Escape::rivalCallback, this, std::placeholders::_1));
        abort_escape = false;

        node->get_parameter("scan_radius", scan_radius);
    }

    // New timer callback function
    void Escape::targetUpdateCallback() {
        // Check if we have the necessary data
        if(!is_active) {
            return;
        }
        if (costmap.data.empty()) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000, "Costmap not yet received, can't update target");
            return;
        }
        
        if (!nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000, "Cannot get current pose, can't update target");
            return;
        }
        
        // Find a new target point
        geometry_msgs::msg::Pose new_target = findTargetPoint();
        
        // Only update if the new target is different or better
        if (new_target.position.x != robotPose.pose.position.x || 
            new_target.position.y != robotPose.pose.position.y) {
            
            double old_cost = getOneGridCost(target_point.position.x, target_point.position.y);
            double new_cost = getOneGridCost(new_target.position.x, new_target.position.y);
            
            if (new_cost < old_cost) {
                RCLCPP_INFO(logger_, "Updating target from (%f, %f) to (%f, %f)", 
                    target_point.position.x, target_point.position.y,
                    new_target.position.x, new_target.position.y);
                target_point = new_target;
            }
        }
    }
    
    Status Escape::onRun(const std::shared_ptr<const EscapeAction::Goal> command){
        if(!nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)){
            return Status::FAILED;
        }
        (void) command;
        map_x = robotPose.pose.position.x * 100;
        map_y = robotPose.pose.position.y * 100;
        rival_x = rivalPose.pose.pose.position.x * 100;
        rival_y = rivalPose.pose.pose.position.y * 100;
        map_width = costmap.info.width;
        map_height = costmap.info.height;
        is_active = true;
        return Status::SUCCEEDED;
    }

    void Escape::costmapCallback(const nav_msgs::msg::OccupancyGrid& msg){
        costmap = msg;
    }

    void Escape::rivalCallback(const nav_msgs::msg::Odometry& msg){
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
                    
                    if (cost < lowest_cost && !outOfBound(world_x, world_y)) {
                        lowest_cost = cost;
                        best_point.position.x = world_x;
                        best_point.position.y = world_y;
                        best_point.position.z = 0.0;
                    }
                }
            }
            
            // If we found a better point than robot position for this radius,
            // return it instead of checking larger radii
            if (lowest_cost < robot_cost) {
                return best_point;
            }
        }
        
        if (lowest_cost < 100.0) {
            abort_escape = false;
        } else {
            RCLCPP_INFO(logger_, "No suitable target point found, staying in place");
            abort_escape = true;
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

        double dx = x - this->robotPose.pose.position.x;
        double dy = y - this->robotPose.pose.position.y;
        double dist = hypot(dx, dy);
        double first_ang_diff = atan2(dy, dx);
        
        double cur_linear_kp = 5;
        double linear_max_vel = 0.4;
        max_vel = std::min(dist * cur_linear_kp, linear_max_vel);

        // Velocity in global frame
        double vel_x_global = max_vel * cos(first_ang_diff);
        double vel_y_global = max_vel * sin(first_ang_diff);

        // Get yaw from robot's orientation (quaternion)
        tf2::Quaternion q(
            this->robotPose.pose.orientation.x,
            this->robotPose.pose.orientation.y,
            this->robotPose.pose.orientation.z,
            this->robotPose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);  // extract yaw angle

        // Rotate global velocity to robot frame
        vel_x =  cos(yaw) * vel_x_global + sin(yaw) * vel_y_global;
        vel_y = -sin(yaw) * vel_x_global + cos(yaw) * vel_y_global;

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
        if(!nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)){
            stopRobot();
            is_active = false;
            return Status::FAILED;
        }

        if(isEscape() || outOfBound(robotPose.pose.position.x, robotPose.pose.position.y)){
            stopRobot();
            is_active = false;
            RCLCPP_INFO(logger_, "Escape successfully");
            return Status::SUCCEEDED;
        }
        
        target_point = findTargetPoint();
        if(abort_escape){
            stopRobot();
            is_active = false;
            abort_escape = false;
            return Status::FAILED;
        }
        
        auto cmd_vel = makeMove(target_point.position.x, target_point.position.y);
        vel_pub_->publish(std::move(cmd_vel));
        return Status::RUNNING;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Escape, nav2_core::Behavior)