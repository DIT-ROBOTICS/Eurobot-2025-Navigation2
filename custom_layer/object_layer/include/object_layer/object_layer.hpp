#ifndef OBJECT_LAYER_HPP_
#define OBJECT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/msg/poseStamped.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <deque>
#include <iostream>
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_gemotry_msgs.h"
#include <cmath>
#include <algorithm>


namespace Object_costmap_plugin {
    class ObjectLayer : public nav2_costmap_2d::CostmapLayer {
        public:
            ObjectLayer(){}
            ~ObjectLayer(){}

            void onInitialize() override;
            void updateBounds(
                double robot_x, double robot_y, double robot_yaw, 
                double *min_x, double *min_y, double *max_x, double *max_y) override;
            void updateCosts(
                nav2_costmap_2d::Costmap2D &master_grid, 
                int min_i, int min_j, int max_i, int max_j) override;
            bool isClearable() override;
            void reset() override;
            
            void ExpandPointWithRectangle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius, geometry_msgs::msg::PoseStamped object);
            void ExpandPointWithCircle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius);
            // data processes
            void columnPoseArrayCallback(const geometry_msgs::PoseArray::SharedPtr object_poseArray);
            void boardPoseArrayCallback(const geometry_msgs::PoseArray::SharedPtr object_poseArray);
        private:
            std::deque<geometry_msgs::msg::PoseStamped> columnList;
            std::deque<geometry_msgs::msg::PoseStamped> boardList;
            rclcpp::Subscription<geometry_msgs::PoseArray>::SharedPtr column_poseArray_sub;
            rclcpp::Subscription<geometry_msgs::PoseArray>::SharedPtr board_poseArray_sub;

            int model_size_ = 22;
            double x_cov_threshold_, y_cov_threshold_, R_sq_threshold_;
            int reset_timeout_threshold_;
            double object_inscribed_radius_;
            double halted_inflation_radius_, wandering_inflation_radius_, moving_inflation_radius_, unknown_inflation_radius_;
            double halted_cost_scaling_factor_, wandering_cost_scaling_factor_, moving_cost_scaling_factor_, unknown_cost_scaling_factor_;
            double max_extend_length_, cov_range_max_, cov_range_min_;
            double inscribed_radius_rate_, inflation_radius_rate_;
            int reset_timeout_ = 0;
            double min_x_ = 0.0, min_y_ = 0.0, max_x_ = 3.0, max_y_ = 2.0;
            double board_width = 0.4, board_height = 0.1;

    };
} // namespace Object_costmap_plgin

#endif  // OBJECT_LAYER_HPP_