#include "object_layer/object_layer.hpp"

namespace Object_costmap_plugin {

    void ObjectLayer::onInitialize(){
        RCLCPP_INDO(
            rclcpp::get_logger("ObjectLayer"), 
            "Initializing ObjectLayer");
        
        enabled_ = true;
        current_ = true;

        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("model_size", rclcpp::ParameterValue(22));
        declareParameter("x_cov_threshold", rclcpp::ParameterValue(0.01));
        declareParameter("y_cov_threshold", rclcpp::ParameterValue(0.01));
        declareParameter("R_sq_threshold", rclcpp::ParameterValue(0.85));
        declareParameter("reset_timeout_threshold", rclcpp::ParameterValue(40));
        declareParameter("object_inscribed_radius_", rclcpp::ParameterValue(0.22));
        declareParameter("halted_inflation_radius_", rclcpp::ParameterValue(0.4));
        declareParameter("wandering_inflation_radius_", rclcpp::ParameterValue(0.5));
        declareParameter("moving_inflation_radius_", rclcpp::ParameterValue(0.43));
        declareParameter("unknown_inflation_radius_", rclcpp::ParameterValue(0.55));
        declareParameter("halted_cost_scaling_factor_", rclcpp::ParameterValue(10.0));
        declareParameter("wandering_cost_scaling_factor_", rclcpp::ParameterValue(3.0));
        declareParameter("moving_cost_scaling_factor_", rclcpp::ParameterValue(11.0));
        declareParameter("unknown_cost_scaling_factor_", rclcpp::ParameterValue(3.0));
        declareParameter("max_extend_length_", rclcpp::ParameterValue(0.5));
        declareParameter("cov_range_max_", rclcpp::ParameterValue(sqrt(0.0029)));
        declareParameter("cov_range_min_", rclcpp::ParameterValue(sqrt(0.0002)));
        declareParameter("inscribed_radius_rate_", rclcpp::ParameterValue(0.99));
        declareParameter("inflation_radius_rate_", rclcpp::ParameterValue(1.005));

        auto node = node_.lock();
        if(!node){
            throw std::runtime_error{"Failed to lock node"};
        }
        node->get_parameter(name_ + "." + "enabled", enabled_);
        node->get_parameter(name_ + "." + "model_size", model_size_);
        node->get_parameter(name_ + "." + "x_cov_threshold", x_cov_threshold_);
        node->get_parameter(name_ + "." + "y_cov_threshold", y_cov_threshold_);
        node->get_parameter(name_ + "." + "R_sq_threshold", R_sq_threshold_);
        node->get_parameter(name_ + "." + "reset_timeout_threshold", reset_timeout_threshold_);
        node->get_parameter(name_ + "." + "object_inscribed_radius_", object_inscribed_radius_);
        node->get_parameter(name_ + "." + "halted_inflation_radius_", halted_inflation_radius_);
        node->get_parameter(name_ + "." + "wandering_inflation_radius_", wandering_inflation_radius_);
        node->get_parameter(name_ + "." + "moving_inflation_radius_", moving_inflation_radius_);
        node->get_parameter(name_ + "." + "unknown_inflation_radius_", unknown_inflation_radius_);
        node->get_parameter(name_ + "." + "halted_cost_scaling_factor_", halted_cost_scaling_factor_);
        node->get_parameter(name_ + "." + "wandering_cost_scaling_factor_", wandering_cost_scaling_factor_);
        node->get_parameter(name_ + "." + "moving_cost_scaling_factor_", moving_cost_scaling_factor_);
        node->get_parameter(name_ + "." + "unknown_cost_scaling_factor_", unknown_cost_scaling_factor_);
        node->get_parameter(name_ + "." + "max_extend_length_", max_extend_length_);
        node->get_parameter(name_ + "." + "cov_range_max_", cov_range_max_);
        node->get_parameter(name_ + "." + "cov_range_min_", cov_range_min_);
        node->get_parameter(name_ + "." + "inscribed_radius_rate_", inscribed_radius_rate_);
        node->get_parameter(name_ + "." + "inflation_radius_rate_", inflation_radius_rate_);

        column_poseArray_sub = node->create_subscription<geometry_msgs::PoseArray>(
            "/column_pose_array", 100, std::bind(&ObjectLayer::columnPoseArrayCallback, this, std::placeholders::_1));
        board_poseArray_sub = node->create_subscription<geometry_msgs::PoseArray>(
            "/board_pose_array", 100, std::bind(&ObjectLayer::boardPoseArrayCallback, this, std::placeholders::_1));
        
        columnList.clear();
        boardList.clear();
        
    }

    void ObjectLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                   double *min_x, double *min_y, double *max_x, double *max_y){
        // Assuming min_x_, min_y_, max_x_, and max_y_ are inherited member variables.
        *min_x = std::min(min_x_, *min_x);
        *min_y = std::min(min_y_, *min_y);
        *max_x = std::max(max_x_, *max_x);
        *max_y = std::max(max_y_, *max_y);
    }

    void ObjectLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j){
        if(!enabled_){
            return;
        }
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
        for(auto object : columnList){
            ExpandPointWithCircle(object.pose.position.x, object.pose.position.y, nav2_costmap_2d::LETHAL_OBSTACLE, object_inscribed_radius_, 1.0, 0.0);
        }
        for(auto object : boardList){
            ExpandPointWithRectangle(object.pose.position.x, object.pose.position.y, nav2_costmap_2d::LETHAL_OBSTACLE, object_inscribed_radius_, 1.0, 0.0, object);
        }
        updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
    }

    bool ObjectLayer::isClearable(){
        return true;
    }

    void ObjectLayer::reset(){
        enabled_ = true;
        current_ = true;
        columnList.clear();
        boardList.clear();
        reset_timeout_ = 0;
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
        RCLCPP_WARN(
            rclcpp::get_logger("ObjectLayer"), 
            "Resetting ObjectLayer");
    }

    void ObjectLayer::columnPoseArrayCallback(const geometry_msgs::PoseArray::SharedPtr object_poseArray){
        columnList.clear();
        for(auto pose : object_poseArray->poses){
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.pose = pose;
            columnList.push_back(poseStamped);
        }
    }

    void ObjectLayer::boardPoseArrayCallback(const geometry_msgs::PoseArray::SharedPtr object_poseArray){
        boardList.clear();
        for(auto pose : object_poseArray->poses){
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.pose = pose;
            boardList.push_back(poseStamped);
        }
    } 

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

    void ObjectLayer::ExpandPointWithRectangle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius, geometry_msgs::msg::PoseStamped object)
    {
        // Get the yaw from the object's orientation (adjust to get desired rectangle orientation).
        double yaw = tf2::getYaw(object.pose.orientation);
        
        // Half dimensions of rectangle, as set in object_layer.hpp.
        double halfWidth  = board_width / 2.0;
        double halfHeight = board_height / 2.0;

        // Compute the 4 vertices of the rectangle around (x, y)
        // The rectangle in its local frame has vertices at:
        // (-halfWidth, -halfHeight), (-halfWidth, halfHeight), (halfWidth, halfHeight), (halfWidth, -halfHeight)
        // Rotate them by yaw and translate by (x, y):
        double cosYaw = cos(yaw);
        double sinYaw = sin(yaw);
        double vx[4], vy[4];
        vx[0] = x + (-halfWidth * cosYaw - (-halfHeight) * sinYaw);
        vy[0] = y + (-halfWidth * sinYaw + (-halfHeight) * cosYaw);
        vx[1] = x + (-halfWidth * cosYaw - ( halfHeight) * sinYaw);
        vy[1] = y + (-halfWidth * sinYaw + ( halfHeight) * cosYaw);
        vx[2] = x + ( halfWidth * cosYaw - ( halfHeight) * sinYaw);
        vy[2] = y + ( halfWidth * sinYaw + ( halfHeight) * cosYaw);
        vx[3] = x + ( halfWidth * cosYaw - (-halfHeight) * sinYaw);
        vy[3] = y + ( halfWidth * sinYaw + (-halfHeight) * cosYaw);
        
        // Determine bounding box of the rotated rectangle.
        double min_tx = *std::min_element(vx, vx+4);
        double max_tx = *std::max_element(vx, vx+4);
        double min_ty = *std::min_element(vy, vy+4);
        double max_ty = *std::max_element(vy, vy+4);

        // Iterate over the bounding box and check if each world point is inside the rotated rectangle.
        // Transform the point (px, py) into the rectangle's local coordinate frame.
        for (double px = min_tx; px <= max_tx; px += resolution_) {
            for (double py = min_ty; py += max_ty; py += resolution_) {
                // Translate point relative to rectangle center (x,y)
                double dx = px - x;
                double dy = py - y;
                // Rotate by -yaw: this aligns the rectangle with the axes.
                double local_x = dx * cosYaw + dy * sinYaw;
                double local_y = -dx * sinYaw + dy * cosYaw;

                // Check if the local point is inside the axis-aligned rectangle.
                if (std::abs(local_x) <= halfWidth && std::abs(local_y) <= halfHeight) {
                    unsigned int mx, my;
                    if (worldToMap(px, py, mx, my)) {
                        // Compute distance from the center (x,y) - you may adjust this calculation if needed.
                        double distance = std::sqrt((px - x) * (px - x) + (py - y) * (py - y));
                        double cost = std::ceil(252 * std::exp(-CostScalingFactor * (distance - InscribedRadius)));
                        cost = std::max(std::min(cost, MaxCost), 0.0);
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
        
}

#include "pluginlib/class_list_macros
PLUGINLIB_EXPORT_CLASS(Object_costmap_plgin::ObjectLayer, nav2_costmap_2d::Layer)