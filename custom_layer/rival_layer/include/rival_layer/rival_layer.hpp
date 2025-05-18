#ifndef RIVAL_LAYER_HPP_
#define RIVAL_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>

// Circular Queue for rival's path      |front| ____ <--- ____ |rear|
class CircularQueue {
    public:
        CircularQueue() {}
        ~CircularQueue() {
            delete[] queue_;
        }

        void init(int size) {
            size_ = size;
                
            queue_ = new std::pair<double, double>[size_];
            for(int i = 0; i < size_; i++) {
                queue_[i] = std::make_pair(0.0, 0.0);
            }
        }

        void push(std::pair<double, double> element) {
            rear_ = (rear_ + 1) % size_;
            queue_[rear_] = element;
            if(rear_ == front_ && !first_cycle_) front_ = (front_ + 1) % size_;
            first_cycle_ = false;
        }

        void reset() {
            front_ = 0;
            rear_ = size_ - 1;
            first_cycle_ = true;
            for(int i = 0; i < size_; i++) {
                queue_[i] = std::make_pair(0.0, 0.0);
            }
        }

        std::pair<double, double> get(int index) {
            return queue_[(front_ + index) % size_];
        }

        bool isFull() {
            if(!first_cycle_)   return (rear_ + 1) % size_ == front_;
            else                return false;
        }

    private:
        int size_ = 22;
        int front_ = 0;
        int rear_ = size_ - 1;
        bool first_cycle_ = true;
        std::pair<double, double> *queue_;
};

namespace custom_path_costmap_plugin {
    class RivalLayer : public nav2_costmap_2d::CostmapLayer {
        public:
            RivalLayer() {}
            ~RivalLayer() {
                rival_path_.~CircularQueue();
            }

            // Functions from Layers
            void onInitialize() override;
            void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y) override;
            void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
            bool isClearable() override;
            void reset() override;
            
            // Functions from Lifecycle Nodes
            void activate() override;
            void deactivate() override;
            
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_service_;
            void handleSetMode(
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            bool mode_param;

        private:
            // Model size for rival's statistics
            int model_size_ = 22;
            // Thresholds for evaluating rival's state and prediction
            double x_cov_threshold_, y_cov_threshold_;
            double R_sq_threshold_;
            // Timeout for reset the costmap
            int reset_timeout_threshold_;
            // Parameters for expansion
            double rival_inscribed_radius_;
            double halted_inflation_radius_, wandering_inflation_radius_, moving_inflation_radius_, unknown_inflation_radius_;
            double halted_cost_scaling_factor_, wandering_cost_scaling_factor_, moving_cost_scaling_factor_, unknown_cost_scaling_factor_;
            double max_extend_length_, cov_range_max_, cov_range_min_, vel_range_max_, vel_range_min_;
            double inscribed_radius_rate_, inflation_radius_rate_;
            // Debug mode
            int debug_mode_;    // 0: off, 1: Print rival state change only, 2: Print rival state change and statistics, 3: Print everything continuously

            // Variables for boundaries
            double min_x_ = 0.0, min_y_ = 0.0, max_x_ = 3.0, max_y_ = 2.0;

            // Variables for statistics calculation
            double rival_x_sum_ = 0.0, rival_y_sum_ = 0.0;
            double rival_x_sq_sum_ = 0.0, rival_y_sq_sum_ = 0.0;
            double rival_xy_sum_ = 0.0;
            double rival_x_mean_ = 0.0, rival_y_mean_ = 0.0;
            double rival_x_var_ = 0.0, rival_y_var_ = 0.0;
            double rival_x_cov_ = 0.0, rival_y_cov_ = 0.0;

            double regression_slope_ = 0.0, regression_intercept_ = 0.0;

            double SSres_ = 0.0, SStot_ = 0.0;
            double R_sq_ = 0.0;
            
            // Variables for rival's pose
            double rival_x_ = 0.0, rival_y_ = 0.0;
            CircularQueue rival_path_;
            double cos_theta_ = 0.0, sin_theta_ = 0.0;

            double v_from_localization_x_ = 0.0;
            double v_from_localization_y_ = 0.0;
            
            int direction_ = 1;
            double rival_distance_;
            double vel_factor_;
            double offset_vel_factor_weight_statistic_;
            double expand_vel_factor_weight_statistic_;
            double offset_vel_factor_weight_localization_;
            double expand_vel_factor_weight_localization_;
            double position_offset_;
            double safe_distance_;
            // Enum for rival's state
            enum class RivalState {
                HALTED,
                WANDERING,
                MOVING,
                UNKNOWN
            };
            RivalState rival_state_ = RivalState::UNKNOWN;

            // Function for rival state and path prediction
            double GetRegressionPrediction(double x);
            void PredictRivalPath();

            // Function for statistics calculation
            void UpdateStatistics();
            
            // Functions for costmap expansion
            void ExpandPointWithCircle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius);
            void ExpandLine(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius, double ExtendLength);
            void FieldExpansion(double x, double y);

            // Functions for update radius
            void updateRadius();

            // Rival pose subscibtion
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rival_distance_sub_;
            void rivalDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg);
      
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rival_pose_sub_;
            void rivalPoseCallback(const nav_msgs::msg::Odometry::SharedPtr rival_pose);
            
            bool rival_pose_received_ = false;

            // Timeout for reset the costmap
            int reset_timeout_ = 100;

            // Use stastistics method or not
            bool use_statistic_method_ = false;

            // DEBUG
            RivalState rival_state_prev_ = RivalState::UNKNOWN;
            void PrintRivalState();
    };
}

#endif  // RIVAL_LAYER_HPP_
