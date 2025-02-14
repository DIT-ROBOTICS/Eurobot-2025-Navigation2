#ifndef OBJECT_LAYER_HPP_
#define OBJECT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/PoseStamped.h"
#include <deque>
#include <iostream>

namespace custom_path_costmap_plgin {
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

            void activate() override;
            void deactivate() override;

            void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr object_pose);
            
            void ExpandPointWithTriangle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius);
            void FieldExpansion(double x, double y);
            void ExpandLine(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius, double ExtendLength);
            bool hasObject();

            void ObjectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr object_pose);

        private:
            deque<geometry_msgs::msg::PoseStamped> objectList;

            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub;

    }
}


#endif  // OBJECT_LAYER_HPP_