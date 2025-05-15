#ifndef OBJECT_LAYER_HPP_
#define OBJECT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <deque>
#include <iostream>
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <algorithm>
#include "tf2/utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"




namespace Object_costmap_plugin {
    class ObjectLayer :  public nav2_costmap_2d::CostmapLayer {
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
            
            void ExpandPointWithRectangle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius, geometry_msgs::msg::PoseStamped object, int mode);
            void ExpandPointWithCircle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius);
            // data processes
            void columnPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr object_poseArray);
            void boardPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr object_poseArray);
            void overturnPoseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr object_poseArray);
            void robotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr object_pose);
            void checkClear();    
            bool eliminateObject(geometry_msgs::msg::PoseStamped column);
            bool checkInBox(double x, double y);

        private:
            std::deque<geometry_msgs::msg::PoseStamped> columnList;
            std::deque<geometry_msgs::msg::PoseStamped> boardList;
            std::deque<geometry_msgs::msg::PoseStamped> overturnList;
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr column_poseArray_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr board_poseArray_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr overturn_sub;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub;
            geometry_msgs::msg::PoseStamped robot_pose;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_service_;
            bool mode_param = false;
            void handleSetMode(
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            int clearTimer;
            bool delay_mode;
            std::string base_frame;
            double lower_x_range, upper_x_range;
            double y_range;
            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;   
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            double column_inscribed_radius, board_inscribed_radius;
            double column_inflation_radius, board_inflation_radius;
            double cost_scaling_factor;
            double min_x_ = 0.0, min_y_ = 0.0, max_x_ = 3.0, max_y_ = 2.0;
            double board_width = 0.4, board_height = 0.1;
            double overturn_width = 0.11, overturn_height = 0.075;

    };
} // namespace Object_costmap_plgin

#endif  // OBJECT_LAYER_HPP_