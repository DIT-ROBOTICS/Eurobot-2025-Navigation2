#ifndef PATH_LAYER_HPP_
#define PATH_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// using namespace nav2_costmap_2d;

namespace custom_path_costmap_plugin {
    class PathLayer : public nav2_costmap_2d::CostmapLayer {
        public:
            PathLayer() = default;
            ~PathLayer() = default;

            void onInitialize() override;
            void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y) override;
            void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
            bool isClearable() override;
            void reset() override;

            void activate() override;
            void deactivate() override;

        private:
            double min_x_ = 0.0, min_y_ = 0.0, max_x_ = 3.0, max_y_ = 2.0;
            double rival_x_ = 0.0, rival_y_ = 0.0;
            double rival_x_prev_ = 0.0, rival_y_prev_ = 0.0;

            void ExpandPointWithCircle(double x, double y, double Radius);

            rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rival_pose_sub_;
            void rivalPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr rival_pose);
    };
}

#endif  // PATH_LAYER_HPP_