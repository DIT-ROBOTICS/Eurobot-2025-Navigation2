#include "path_layer/path_layer.hpp"

namespace custom_path_costmap_plugin { 
    // PathLayer class
    void PathLayer::onInitialize() {
        enabled_ = true;
        current_ = true;
        RCLCPP_INFO(
            rclcpp::get_logger("PathLayer"), 
            "Initializing PathLayer");

        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        rival_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/rival_pose", 10, std::bind(&PathLayer::rivalPoseCallback, this, std::placeholders::_1));
    }

    void PathLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, 
        double *min_x, double *min_y, double *max_x, double *max_y) {

        // Update the bounds of the costmap
        *min_x = std::min(min_x_, *min_x);
        *min_y = std::min(min_y_, *min_y);
        *max_x = std::max(max_x_, *max_x);
        *max_y = std::max(max_y_, *max_y);
    }

    void PathLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, 
        int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/) {

        // Check if the layer is enabled
        if (!enabled_) {
            return;
        }

        // Set the rival as a lethal obstacle
        if(rival_x_!=rival_x_prev_ || rival_y_!=rival_y_prev_)  PathLayer::ExpandPointWithCircle(rival_x_, rival_y_, 0.4);

        // Update the costmap with the rival's path
        PathLayer::updateWithMax(master_grid, 0, 0, PathLayer::getSizeInCellsX(), PathLayer::getSizeInCellsY());
        // PathLayer::updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        rival_x_prev_ = rival_x_;
        rival_y_prev_ = rival_y_;
    }

    bool PathLayer::isClearable() {
        return true;
    }

    void PathLayer::reset() {
        enabled_ = false;
        current_ = false;
    }

    void PathLayer::ExpandPointWithCircle(double x, double y, double Radius) {
        for(float r=0.0; r<=Radius; r+=0.01){
            for(int angle = 0; angle < 360; angle+=1) {
                double Rad = angle * M_PI / 180.0;
                double mark_x = x + r * cos(Rad);
                double mark_y = y + r * sin(Rad);

                PathLayer::setCost(int(mark_x*100), int(mark_y*100), nav2_costmap_2d::LETHAL_OBSTACLE*((Radius-(r))/Radius));
            }
        }
    }

    // Subscribe to the rival's pose
    void PathLayer::activate() {
        RCLCPP_INFO(
            rclcpp::get_logger("PathLayer"), 
            "Activating PathLayer");
    }

    void PathLayer::deactivate() {
        RCLCPP_INFO(
            rclcpp::get_logger("PathLayer"), 
            "Deactivating PathLayer");
    }

    void PathLayer::rivalPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr rival_pose) {
        rival_x_ = rival_pose->pose.pose.position.x;
        rival_y_ = rival_pose->pose.pose.position.y;
        PathLayer::resetMaps();
    }

}   // namespace custom_path_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_path_costmap_plugin::PathLayer, nav2_costmap_2d::Layer)