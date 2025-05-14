#include "object_layer/object_layer.hpp"

namespace Object_costmap_plugin {

    void ObjectLayer::onInitialize(){
        RCLCPP_INFO(
            rclcpp::get_logger("ObjectLayer"), 
            "Initializing ObjectLayer");

        enabled_ = true;
        current_ = true;
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
        auto node = node_.lock();
        if(!node){
            throw std::runtime_error{"Failed to lock node"};
        }

        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("column_inscribed_radius", rclcpp::ParameterValue(0.75));
        declareParameter("board_inscribed_radius", rclcpp::ParameterValue(0.01));
        declareParameter("column_inflation_radius", rclcpp::ParameterValue(0.22));
        declareParameter("board_inflation_radius", rclcpp::ParameterValue(0.22));
        declareParameter("cost_scaling_factor", rclcpp::ParameterValue(3.0));
        declareParameter("delay_mode", rclcpp::ParameterValue(false));
        declareParameter("upper_x_range", rclcpp::ParameterValue(0.23));
        declareParameter("lower_x_range", rclcpp::ParameterValue(0.11));
        declareParameter("y_range", rclcpp::ParameterValue(0.22));
        declareParameter("base_frame", rclcpp::ParameterValue("base_footprint"));
        
        node->get_parameter(name_ + "." + "base_frame", base_frame);
        node->get_parameter(name_ + "." + "enabled", enabled_);
        node->get_parameter(name_ + "." + "column_inscribed_radius", column_inscribed_radius);
        node->get_parameter(name_ + "." + "board_inscribed_radius", board_inscribed_radius);
        node->get_parameter(name_ + "." + "column_inflation_radius", column_inflation_radius);
        node->get_parameter(name_ + "." + "board_inflation_radius", board_inflation_radius);
        node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor);
        node->get_parameter(name_ + "." + "delay_mode", delay_mode);
        node->get_parameter(name_ + "." + "upper_x_range", upper_x_range);
        node->get_parameter(name_ + "." + "lower_x_range", lower_x_range);
        node->get_parameter(name_ + "." + "y_range", y_range);
        RCLCPP_INFO(
            rclcpp::get_logger("ObjectLayer"), 
            "ObjectLayer parameter enable %d", enabled_);
        column_poseArray_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
            "/detected/global_center_poses/column", 100, std::bind(&ObjectLayer::columnPoseArrayCallback, this, std::placeholders::_1));
        board_poseArray_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
            "/detected/global_center_poses/platform", 100, std::bind(&ObjectLayer::boardPoseArrayCallback, this, std::placeholders::_1));
        robot_pose_sub = node->create_subscription<nav_msgs::msg::Odometry>(
            "/final_pose_nav", 100, std::bind(&ObjectLayer::robotPoseCallback, this, std::placeholders::_1));
        overturn_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
            "/detected/global_center_poses/overturn", 100, std::bind(&ObjectLayer::overturnPoseArrayCallback, this, std::placeholders::_1));
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
        tf2_buffer_->setUsingDedicatedThread(true);
        clearTimer = 20;
        mode_param = false;
        set_mode_service_ = node->create_service<std_srvs::srv::SetBool>(
            "/object_layer/set_mode", std::bind(&ObjectLayer::handleSetMode, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize robot pose with default values to prevent empty frame_id
        robot_pose.header.frame_id = "map";
        robot_pose.header.stamp = node->now();
        robot_pose.pose.orientation.w = 1.0;  // Identity quaternion

    }

    void ObjectLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                   double *min_x, double *min_y, double *max_x, double *max_y){
        // Assuming min_x_, min_y_, max_x_, and max_y_ are inherited member variables.
        (void) robot_x;
        (void) robot_y;
        (void) robot_yaw;
        *min_x = std::min(min_x_, *min_x);
        *min_y = std::min(min_y_, *min_y);
        *max_x = std::max(max_x_, *max_x);
        *max_y = std::max(max_y_, *max_y);
    }

    void ObjectLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j){
        if(!enabled_){
            return;
        }
        auto node = node_.lock();
        if(mode_param) {
            column_inflation_radius = 0.1;
            board_inflation_radius = 0.1;
        }
        else {
            node->get_parameter(name_ + "." + "column_inflation_radius", column_inflation_radius);
            node->get_parameter(name_ + "." + "board_inflation_radius", board_inflation_radius);
        }

        for(auto object : columnList){
            if(eliminateObject(object)){
                continue;
            }
            else ExpandPointWithCircle(object.pose.position.x, object.pose.position.y, nav2_costmap_2d::LETHAL_OBSTACLE, column_inflation_radius, cost_scaling_factor, column_inscribed_radius);
            updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());

        }
        for(auto object : boardList){
            if(eliminateObject(object)){
                continue;
            }
            else ExpandPointWithRectangle(object.pose.position.x, object.pose.position.y, nav2_costmap_2d::LETHAL_OBSTACLE, board_inflation_radius, cost_scaling_factor, board_inscribed_radius, object, 0);
            updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
        }
        for(auto object : overturnList){
            ExpandPointWithRectangle(object.pose.position.x, object.pose.position.y, nav2_costmap_2d::LETHAL_OBSTACLE, board_inflation_radius, cost_scaling_factor, board_inscribed_radius, object, 1);
            updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
        }
        // updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
        // checkClear();
    }

    void ObjectLayer::handleSetMode(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if(request->data) {
            mode_param = true;
            response->success = true;
            response->message = "ObjectLayer mode set to shrink mode";
        }
        else if(!request->data){
            mode_param = false;
            response->success = true;
            response->message = "ObjectLayer mode set to normal mode";
        }
    }

    void ObjectLayer::checkClear(){
        if(!delay_mode){
            boardList.clear();
            columnList.clear();
        }
        else if(delay_mode){
            if(clearTimer == 0){
                boardList.clear();
                columnList.clear();
                clearTimer = 20;
            }
            clearTimer--;
        }
    }

    bool ObjectLayer::checkInBox(double x, double y){
        if(y < y_range && y > -y_range){
            if(x>= 0 && x < upper_x_range){
                if(x > lower_x_range) return true;
                else return false;
            }
            else if(x < 0 && x > -upper_x_range){
                if(x < -lower_x_range) return true;
                else return false;
            }
            else return false;
            }
        else return false;

    }

    bool ObjectLayer::eliminateObject(geometry_msgs::msg::PoseStamped column){ 
        if (!tf2_buffer_) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ObjectLayer"),
                "TF2 buffer is null, cannot transform poses");
            return false;
        }

        if(column.header.frame_id.empty()){
            column.header.frame_id = "map";
        }

        // // Add timestamp if missing
        // if(column.header.stamp.seconds() == 0) {
        //     auto node = node_.lock();
        //     if (node) {
        //         column.header.stamp = node->now();
        //     }
        // }

        try {
            geometry_msgs::msg::PoseStamped transformed_pose;
            
            // Use zero time to get latest transform
            transformed_pose = tf2_buffer_->transform(
                column, 
                base_frame,
                tf2::TimePointZero,
                column.header.frame_id,
                tf2::durationFromSec(0.1) // 100ms timeout
            );
            
            return checkInBox(transformed_pose.pose.position.x, transformed_pose.pose.position.y);
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(
                rclcpp::get_logger("ObjectLayer"),
                "Transform exception: %s", ex.what());
            return false;
        }
    }

    bool ObjectLayer::isClearable(){
        return true;
    }

    void ObjectLayer::reset(){
        enabled_ = true;
        current_ = true;
        columnList.clear();
        boardList.clear();
        overturnList.clear();
        tf2_buffer_->clear();
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
        RCLCPP_WARN(
            rclcpp::get_logger("ObjectLayer"), 
            "Resetting ObjectLayer");
    }

    void ObjectLayer::overturnPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr object_poseArray){
        overturnList.clear();
        for(auto pose : object_poseArray->poses){
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.pose = pose;
            poseStamped.header.frame_id = "map";
            overturnList.push_back(poseStamped);
        }
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
    }

    void ObjectLayer::robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr object_pose){
        robot_pose.header.frame_id = object_pose->header.frame_id;
        robot_pose.header.stamp = object_pose->header.stamp;
        robot_pose.pose.position.x = object_pose->pose.pose.position.x;
        robot_pose.pose.position.y = object_pose->pose.pose.position.y;
        robot_pose.pose.position.z = object_pose->pose.pose.position.z;
        robot_pose.pose.orientation.x = object_pose->pose.pose.orientation.x;
        robot_pose.pose.orientation.y = object_pose->pose.pose.orientation.y;
        robot_pose.pose.orientation.z = object_pose->pose.pose.orientation.z;
        robot_pose.pose.orientation.w = object_pose->pose.pose.orientation.w;

        // log and check if frame id is set
    }

    void ObjectLayer::columnPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr object_poseArray){
        columnList.clear();
        for(auto pose : object_poseArray->poses){
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.pose = pose;
            poseStamped.header.frame_id = "map";
            columnList.push_back(poseStamped);
        }
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

    }

    void ObjectLayer::boardPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr object_poseArray){
        boardList.clear();
        for(auto pose : object_poseArray->poses){
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.pose = pose;
            poseStamped.header.frame_id = "map";
            boardList.push_back(poseStamped);
        }
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
    }
    // 0.22
    void ObjectLayer::ExpandPointWithCircle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius){
        double maxX = x + InflationRadius;
        double minX = x - InflationRadius;
        double maxY;
        double minY;

        double mark_x = 0.0;
        double mark_y = 0.0;
        unsigned int mx, my;
        double cost;
        double distance;

        for(double currentX = minX; currentX <= maxX; currentX+=resolution_){
            mark_x = currentX;
            maxY = y + sqrt(pow(InflationRadius, 2) - pow(fabs(currentX - x), 2));
            minY = 2 * y - maxY;

            for(double currentY = minY; currentY <= maxY; currentY+=resolution_){
                mark_y = currentY;
                if(worldToMap(mark_x, mark_y, mx, my)){
                    distance = sqrt(pow(fabs(x - currentX), 2) + pow(fabs(y - currentY), 2));
                    cost = ceil(252 * exp(-CostScalingFactor * (distance - InscribedRadius)));
                    cost = std::max(std::min(cost, MaxCost), 0.0);
                    if(getCost(mx, my) != nav2_costmap_2d::NO_INFORMATION){
                        setCost(mx, my, std::max((unsigned char)cost, getCost(mx, my)));
                    } else {
                        setCost(mx, my, cost);
                    }
                }
            }
        }
    }

    void ObjectLayer::ExpandPointWithRectangle(double x, double y, double MaxCost,
                                         double InflationRadius, double CostScalingFactor, double InscribedRadius, 
                                         geometry_msgs::msg::PoseStamped object, int mode)
    {
        // Get rectangle half-dimensions from header-defined values.
        double halfWidth = 0;
        double halfHeight = 0;
        if(mode == 0) {
            halfWidth  = board_width / 2.0;
            halfHeight = board_height / 2.0;
        }
        else if(mode == 1) {
            halfWidth  = overturn_width / 2.0;
            halfHeight = overturn_height / 2.0;
        }
        else {
            RCLCPP_ERROR(
                rclcpp::get_logger("ObjectLayer"), 
                "Invalid mode for ExpandPointWithRectangle");
            return;
        }
        
        // Get the yaw (orientation) from the object's quaternion.
        double siny_cosp = 2.0 * (object.pose.orientation.w * object.pose.orientation.z +
                                  object.pose.orientation.x * object.pose.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (object.pose.orientation.y * object.pose.orientation.y +
                                        object.pose.orientation.z * object.pose.orientation.z);
        double angle = std::atan2(siny_cosp, cosy_cosp);
        (void) angle;
        // RCLCPP_WARN(
        //     rclcpp::get_logger("ObjectLayer"), 
        //     "my angle : %lf", angle);        
        // Precompute sine and cosine of the angle.
        double receivedAngle = object.pose.orientation.x;
        double cosAngle = std::cos(receivedAngle);
        double sinAngle = std::sin(receivedAngle);
        
        unsigned int mx, my;
        double cost = 0.0;
        
        // Define the local bounds including the inflation region.
        double localBoundX = halfWidth + InflationRadius;
        double localBoundY = halfHeight + InflationRadius;
        
        // Use an epsilon so that the loop covers the entire region.
        double epsilon = resolution_ * 0.5;
        
        // Iterate over the inflated local region in the object's frame.
        for (double local_x = -localBoundX; local_x < localBoundX + epsilon; local_x += resolution_) {
            for (double local_y = -localBoundY; local_y < localBoundY + epsilon; local_y += resolution_) {
                // Transform the local point into world coordinates.
                double rotated_x = local_x * cosAngle - local_y * sinAngle;
                double rotated_y = local_x * sinAngle + local_y * cosAngle;
                double world_x = x + rotated_x;
                double world_y = y + rotated_y;
                
                if (worldToMap(world_x, world_y, mx, my)) {
                    // If inside the base rectangle, assign maximum cost.
                    if (std::fabs(local_x) <= halfWidth && std::fabs(local_y) <= halfHeight) {
                        cost = MaxCost;
                    } 
                    else {
                        // Outside the base rectangle: compute the distance from the nearest rectangle border.
                        double overflow_x = std::max(0.0, std::fabs(local_x) - halfWidth);
                        double overflow_y = std::max(0.0, std::fabs(local_y) - halfHeight);
                        double border_distance = std::sqrt(overflow_x * overflow_x + overflow_y * overflow_y);
                        
                        // Only inflate if within the InflationRadius.
                        if (border_distance <= InflationRadius) {
                            cost = std::ceil(252 * std::exp(-CostScalingFactor * (border_distance - InscribedRadius)));
                            cost = std::max(std::min(cost, MaxCost), 0.0);
                        } else {
                            cost = 0.0;
                        }
                    }
                    // Update the costmap cell.
                    if (getCost(mx, my) != nav2_costmap_2d::NO_INFORMATION) {
                        setCost(mx, my, std::max((unsigned char)cost, getCost(mx, my)));
                    } else {
                        setCost(mx, my, cost);
                    }
                }
            }
        }
    }
        
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(Object_costmap_plugin::ObjectLayer, nav2_costmap_2d::Layer)