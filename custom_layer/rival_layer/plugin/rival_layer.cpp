#include "rival_layer/rival_layer.hpp"

namespace custom_path_costmap_plugin { 
    // RivalLayer class
    void RivalLayer::onInitialize() {
        RCLCPP_INFO(
            rclcpp::get_logger("RivalLayer"), 
            "Initializing RivalLayer");

        // Initialize the layer
        enabled_ = true;
        current_ = true;
        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

        // Get the node
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        // Declare the parameters
        declareParameter("enabled", rclcpp::ParameterValue(true));

        declareParameter("model_size", rclcpp::ParameterValue(22));

        declareParameter("x_cov_threshold", rclcpp::ParameterValue(0.01));
        declareParameter("y_cov_threshold", rclcpp::ParameterValue(0.01));
        declareParameter("R_sq_threshold", rclcpp::ParameterValue(0.85));

        declareParameter("reset_timeout_threshold", rclcpp::ParameterValue(40));

        declareParameter("robot_inscribed_radius", rclcpp::ParameterValue(0.22));
        // If given external_rival_data_path, use it to load the rival's data
        declareParameter("external_rival_data_path", rclcpp::ParameterValue(""));
        declareParameter("rival_inscribed_radius", rclcpp::ParameterValue(0.22));

        declareParameter("halted_inflation_radius", rclcpp::ParameterValue(0.1));
        declareParameter("wandering_inflation_radius", rclcpp::ParameterValue(0.1));
        declareParameter("moving_inflation_radius", rclcpp::ParameterValue(0.1));
        declareParameter("unknown_inflation_radius", rclcpp::ParameterValue(0.1));

        declareParameter("halted_cost_scaling_factor", rclcpp::ParameterValue(10.0));
        declareParameter("wandering_cost_scaling_factor", rclcpp::ParameterValue(3.0));
        declareParameter("moving_cost_scaling_factor", rclcpp::ParameterValue(11.0));
        declareParameter("unknown_cost_scaling_factor", rclcpp::ParameterValue(3.0));

        declareParameter("max_extend_length", rclcpp::ParameterValue(0.6));

        declareParameter("cov_range_max", rclcpp::ParameterValue(sqrt(0.0029)));
        declareParameter("cov_range_min", rclcpp::ParameterValue(sqrt(0.0002)));

        declareParameter("vel_range_max", rclcpp::ParameterValue(1.0));
        declareParameter("vel_range_min", rclcpp::ParameterValue(0.05));

        declareParameter("inscribed_radius_rate", rclcpp::ParameterValue(0.99));
        declareParameter("inflation_radius_rate", rclcpp::ParameterValue(1.005));

        declareParameter("debug_mode", rclcpp::ParameterValue(0));

        declareParameter("offset_vel_factor_weight_statistic", rclcpp::ParameterValue(0.42));
        declareParameter("expand_vel_factor_weight_statistic", rclcpp::ParameterValue(0.20));
        declareParameter("offset_vel_factor_weight_localization", rclcpp::ParameterValue(0.42));
        declareParameter("safe_distance", rclcpp::ParameterValue(0.5));
        declareParameter("expand_vel_factor_weight_localization", rclcpp::ParameterValue(0.20));
        declareParameter("use_statistic_method", rclcpp::ParameterValue(false));
    
        // Get the parameters
        node->get_parameter(name_ + "." + "enabled", enabled_);

        node->get_parameter(name_ + "." + "model_size", model_size_);

        node->get_parameter(name_ + "." + "x_cov_threshold", x_cov_threshold_);
        node->get_parameter(name_ + "." + "y_cov_threshold", y_cov_threshold_);
        node->get_parameter(name_ + "." + "R_sq_threshold", R_sq_threshold_);

        node->get_parameter(name_ + "." + "reset_timeout_threshold", reset_timeout_threshold_);

        double robot_inscribed_radius;
        node->get_parameter(name_ + "." + "robot_inscribed_radius", robot_inscribed_radius);
        std::string external_rival_data_path;
        node->get_parameter(name_ + "." + "external_rival_data_path", external_rival_data_path);
        node->get_parameter(name_ + "." + "rival_inscribed_radius", rival_inscribed_radius_);
        if (!external_rival_data_path.empty()) {
            try {
                YAML::Node config = YAML::LoadFile(external_rival_data_path);
                if (config["rival_parameters"] && config["rival_parameters"]["rival_inscribed_radius"]) {
                    rival_inscribed_radius_ = config["rival_parameters"]["rival_inscribed_radius"].as<double>();
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("RivalLayer"), "rival_inscribed_radius not found in YAML file, using default value");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(rclcpp::get_logger("RivalLayer"), "Failed to load YAML file: %s, using default value", e.what());
            }
        } 
        // TODO: Refactor the implementation code for rival_inscribed_radius_ to avoid misleading the variable name
        rival_inscribed_radius_ += robot_inscribed_radius;
        RCLCPP_INFO(
            rclcpp::get_logger("RivalLayer"), 
            "\033[1;35mRival inscribed radius is set to %f\033[0m", rival_inscribed_radius_);

        node->get_parameter(name_ + "." + "halted_inflation_radius", halted_inflation_radius_);
        node->get_parameter(name_ + "." + "wandering_inflation_radius", wandering_inflation_radius_);
        node->get_parameter(name_ + "." + "moving_inflation_radius", moving_inflation_radius_);
        node->get_parameter(name_ + "." + "unknown_inflation_radius", unknown_inflation_radius_);
        // TODO: Refactor the implementation code for inflation_radius_ to avoid misleading the variable name
        halted_inflation_radius_ += rival_inscribed_radius_;
        wandering_inflation_radius_ += rival_inscribed_radius_;
        moving_inflation_radius_ += rival_inscribed_radius_;
        unknown_inflation_radius_ += rival_inscribed_radius_;

        node->get_parameter(name_ + "." + "halted_cost_scaling_factor", halted_cost_scaling_factor_);
        node->get_parameter(name_ + "." + "wandering_cost_scaling_factor", wandering_cost_scaling_factor_);
        node->get_parameter(name_ + "." + "moving_cost_scaling_factor", moving_cost_scaling_factor_);
        node->get_parameter(name_ + "." + "unknown_cost_scaling_factor", unknown_cost_scaling_factor_);

        node->get_parameter(name_ + "." + "max_extend_length", max_extend_length_);

        node->get_parameter(name_ + "." + "cov_range_max", cov_range_max_);
        node->get_parameter(name_ + "." + "cov_range_min", cov_range_min_);

        node->get_parameter(name_ + "." + "vel_range_max", vel_range_max_);
        node->get_parameter(name_ + "." + "vel_range_min", vel_range_min_);

        node->get_parameter(name_ + "." + "inscribed_radius_rate", inscribed_radius_rate_);
        node->get_parameter(name_ + "." + "inflation_radius_rate", inflation_radius_rate_);

        node->get_parameter(name_ + "." + "debug_mode", debug_mode_);

        node->get_parameter(name_ + "." + "offset_vel_factor_weight_statistic", offset_vel_factor_weight_statistic_);
        node->get_parameter(name_ + "." + "expand_vel_factor_weight_statistic", expand_vel_factor_weight_statistic_);
        node->get_parameter(name_ + "." + "offset_vel_factor_weight_localization", offset_vel_factor_weight_localization_);
        node->get_parameter(name_ + "." + "expand_vel_factor_weight_localization", expand_vel_factor_weight_localization_);
        
        node->get_parameter(name_ + "." + "safe_distance", safe_distance_);

        node->get_parameter(name_ + "." + "use_statistic_method", use_statistic_method_);

        // Subscribe to the rival's pose
        rival_pose_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/rival/final_pose", 100, std::bind(&RivalLayer::rivalPoseCallback, this, std::placeholders::_1));
        rival_distance_sub_ = node->create_subscription<std_msgs::msg::Float64>(
            "/rival_distance", 100, std::bind(&RivalLayer::rivalDistanceCallback, this, std::placeholders::_1));
        
        set_mode_service_ = node->create_service<std_srvs::srv::SetBool>(
            "/rival_layer/set_mode", std::bind(&RivalLayer::handleSetMode, this, std::placeholders::_1, std::placeholders::_2));
        mode_param = 0;

        // Initialize the queue
        rival_path_.init(model_size_);
    }

    void RivalLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, 
        double *min_x, double *min_y, double *max_x, double *max_y) {

        // Update the bounds of the costmap
        *min_x = std::min(min_x_, *min_x);
        *min_y = std::min(min_y_, *min_y);
        *max_x = std::max(max_x_, *max_x);
        *max_y = std::max(max_y_, *max_y);
    }

    void RivalLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid, 
        int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/) {

        // Check if the layer is enabled
        if (!enabled_) {
            return;
        }

        updateRadius();
        
        // Set the rival as a lethal obstacle & Update the costmap with the rival's path
        if(rival_pose_received_) {
            resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
            FieldExpansion(rival_x_, rival_y_);

            rival_pose_received_ = false;
            reset_timeout_ = 0;
        } else {
            reset_timeout_++;
        }

        if(reset_timeout_ >= reset_timeout_threshold_)  reset();

        updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
    }

    bool RivalLayer::isClearable() {
        return true;
    }

    void RivalLayer::handleSetMode(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if(request->data) {
            mode_param = 1;
            response->success = true;
            response->message = "RivalLayer is in shrink mode";
        } else {
            mode_param = 0;
            response->success = true;
            response->message = "RivalLayer is in default mode";
        }
    }

    void RivalLayer::reset() {
        enabled_ = true;
        current_ = true;

        rival_x_ = 0.0;
        rival_y_ = 0.0;

        rival_path_.reset();
        
        rival_x_sum_ = 0.0;
        rival_y_sum_ = 0.0;
        rival_x_sq_sum_ = 0.0;
        rival_y_sq_sum_ = 0.0;
        rival_xy_sum_ = 0.0;
        rival_x_mean_ = 0.0;
        rival_y_mean_ = 0.0;
        rival_x_var_ = 0.0;
        rival_y_var_ = 0.0;
        rival_x_cov_ = 0.0;
        rival_y_cov_ = 0.0;
        regression_slope_ = 0.0;
        regression_intercept_ = 0.0;
        R_sq_ = 0.0;
        SSres_ = 0.0;
        SStot_ = 0.0;
        rival_state_ = RivalState::UNKNOWN;
        rival_state_prev_ = RivalState::UNKNOWN;

        rival_pose_received_ = false;

        resetMapToValue(0, 0, getSizeInCellsX(), getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);

        reset_timeout_ = 0;
    }

    void RivalLayer::PredictRivalPath() {
        // Debug
        if(debug_mode_ == 1 || debug_mode_ == 2 || debug_mode_ == 3)    PrintRivalState();

        // Pedict the rival's state
        UpdateStatistics();

        if(use_statistic_method_) {
            if(rival_path_.isFull()) {
                if(rival_x_cov_ < x_cov_threshold_ && rival_y_cov_ < y_cov_threshold_) {
                    rival_state_ = RivalState::HALTED;

                } else if(R_sq_ < R_sq_threshold_) {
                    rival_state_ = RivalState::WANDERING;
                
                } else {
                    rival_state_ = RivalState::MOVING;
                }
            } else {
                rival_state_ = RivalState::UNKNOWN;
                return;
            }
        } else {
            // First, we check the rival velocity
            if(v_from_localization_x_ < 0.05 && v_from_localization_y_ < 0.05) {
                rival_state_ = RivalState::HALTED;
            } else {
                rival_state_ = RivalState::MOVING;
            }
            // Then, we double check the CoV to filter the unwanted velocity
            // if(rival_path_.isFull()) {
            //     if(rival_x_cov_ < x_cov_threshold_ && rival_y_cov_ < y_cov_threshold_) {
            //         rival_state_ = RivalState::HALTED;
            //     }
            // }
        }
    }

    double RivalLayer::GetRegressionPrediction(double x) {
        return regression_slope_ * x + regression_intercept_;
    }

    void RivalLayer::UpdateStatistics() {
        // Calculate the rival's statistics
        if(rival_path_.isFull()) {
            // Calculate the CoV
            rival_x_mean_ = rival_x_sum_ / model_size_;
            rival_y_mean_ = rival_y_sum_ / model_size_;
            
            // Check if the variance is zero
            // ? It might lead to -nan beacause of the accuracy of the floating point 
            if(fabs(rival_x_mean_ * rival_x_mean_ - rival_x_sq_sum_ / model_size_) < 1e-6) {
                rival_x_var_ = 0.0;
            } else {
                rival_x_var_ = sqrt(rival_x_sq_sum_ / model_size_ - rival_x_mean_ * rival_x_mean_);
            }
            
            if(fabs(rival_y_mean_ * rival_y_mean_ - rival_y_sq_sum_ / model_size_) < 1e-6) {
                rival_y_var_ = 0.0;
            } else {
                rival_y_var_ = sqrt(rival_y_sq_sum_ / model_size_ - rival_y_mean_ * rival_y_mean_);
            }

            rival_x_cov_ = rival_x_var_ / rival_x_mean_;
            rival_y_cov_ = rival_y_var_ / rival_y_mean_;

            // Check if the regression line is not vertical
            if(fabs(model_size_ * rival_x_sq_sum_ - rival_x_sum_ * rival_x_sum_) < 1e-6) {
                // RCLCPP_WARN(
                //     rclcpp::get_logger("RivalLayer"), 
                //     "Regression line is vertical, using  approximation !!!");

                regression_slope_ = 1e6;
                regression_intercept_ = (rival_y_sum_ - regression_slope_ * rival_x_sum_) / model_size_;

                R_sq_ = 1.0;

                return;
            }

            // Calculate the regression line

            if(use_statistic_method_) {
                regression_slope_ = (model_size_ * rival_xy_sum_ - rival_x_sum_ * rival_y_sum_) / (model_size_ * rival_x_sq_sum_ - rival_x_sum_ * rival_x_sum_);
            } else {
                if(v_from_localization_x_)  regression_slope_ = v_from_localization_y_ / v_from_localization_x_;
                else regression_slope_ = 1e6;
            }
          
            regression_intercept_ = (rival_y_sum_ - regression_slope_ * rival_x_sum_) / model_size_;

            if(use_statistic_method_) {
                cos_theta_ = 1 / sqrt(1 + pow(regression_slope_, 2));
                sin_theta_ = regression_slope_ / sqrt(1 + pow(regression_slope_, 2));
                direction_ = (rival_path_.get(model_size_ - 1).first - rival_path_.get(0).first) > 0 ? 1 : -1;
            } else {
                cos_theta_ = 1 / sqrt(1 + pow(v_from_localization_y_ / v_from_localization_x_, 2));
                sin_theta_ = (v_from_localization_y_/v_from_localization_x_) / sqrt(1 + pow(v_from_localization_y_ / v_from_localization_x_, 2));
                direction_ = v_from_localization_x_ > 0 ? 1 : -1;
            }

            for(int i=0; i<model_size_; i++) {
                SSres_ += pow(rival_path_.get(i).second - GetRegressionPrediction(rival_path_.get(i).first), 2);
                SStot_ += pow(rival_path_.get(i).second - rival_y_mean_, 2);
            } 

            R_sq_ = 1 - SSres_ / SStot_;

        } else {
            if(debug_mode_ == 3) {
                RCLCPP_WARN(
                    rclcpp::get_logger("RivalLayer"), 
                    "Not enough data for statistics calculation");
            }
            return;
        }
    }

    void RivalLayer::ExpandPointWithCircle(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius) {
        
        //RCLCPP_INFO(logger_,"vel_factor is [%lf]", vel_factor);
        if(rival_state_ == RivalState::MOVING){
            x += position_offset_ * cos_theta_ * direction_;
            y += position_offset_ * sin_theta_ * direction_;
        }
        
        double MaxX = x + InflationRadius;
        double MinX = x - InflationRadius;
        double MaxY;
        double MinY;

        double mark_x = 0.0;
        double mark_y = 0.0;
        unsigned int mx;
        unsigned int my;

        double cost;
        double Distance;

        for (double currentPointX = MinX; currentPointX <= MaxX; currentPointX += resolution_) {
            mark_x = currentPointX;
            MaxY = y + sqrt(pow(InflationRadius, 2) - pow(fabs(currentPointX - x), 2));
            MinY = 2 * y - MaxY;

            for (double currentPointY = MinY; currentPointY <= MaxY; currentPointY += resolution_) {
                mark_y = currentPointY;
                if (worldToMap(mark_x, mark_y, mx, my)) {
                    Distance = sqrt(pow(fabs(x - currentPointX), 2) + pow(fabs(y - currentPointY), 2));

                    if (Distance <= InscribedRadius) {
                        cost = MaxCost;
                    } else {
                        cost = ceil(252 * exp(-CostScalingFactor * (Distance - InscribedRadius)));
                        cost = std::max(std::min(cost, MaxCost), 0.0);
                    }
                    
                    if (getCost(mx, my) != nav2_costmap_2d::NO_INFORMATION) {
                        setCost(mx, my, std::max((unsigned char)cost, getCost(mx, my)));
                    } else {
                        setCost(mx, my, cost);
                    }
                }
            }
        }
    }

    void RivalLayer::ExpandLine(double x, double y, double MaxCost, double InflationRadius, double CostScalingFactor, double InscribedRadius, double ExtendLength) {
        double mark_x = 0/*position_offset_ * cos_theta_ * direction_*/;
        double mark_y = 0/*position_offset_ * sin_theta_ * direction_*/;
        unsigned int mx, my;
        int goal_steps = ExtendLength / resolution_;
        
        
        if(goal_steps == 0 && worldToMap(x, y, mx, my) && rival_distance_ < 0.75) {
            ExpandPointWithCircle(x, y, MaxCost, InflationRadius, CostScalingFactor, InscribedRadius);
        }

        for(int i=0; i<goal_steps; i++) { 
            mark_x += resolution_ * cos_theta_ * direction_;
            mark_y += resolution_ * sin_theta_ * direction_;

            if(worldToMap(x + mark_x, y + mark_y, mx, my)) {
                ExpandPointWithCircle(x + mark_x, y + mark_y, MaxCost, InflationRadius, CostScalingFactor, InscribedRadius);
            }

            // Decrease the InscribedRadius
            InscribedRadius *= inscribed_radius_rate_;
            InflationRadius *= inflation_radius_rate_;
        }
    }

    void RivalLayer::FieldExpansion(double x, double y) {
        // Update Prediction
        PredictRivalPath();

        // Expand the costmap based on the rival's state
        switch (rival_state_) {
            case RivalState::HALTED:
                ExpandPointWithCircle(x, y, nav2_costmap_2d::MAX_NON_OBSTACLE, halted_inflation_radius_, halted_cost_scaling_factor_, rival_inscribed_radius_);
                break;
            
            case RivalState::WANDERING:
                ExpandPointWithCircle(x, y, nav2_costmap_2d::MAX_NON_OBSTACLE, wandering_inflation_radius_, wandering_cost_scaling_factor_, rival_inscribed_radius_);
                break;

            case RivalState::MOVING:
                if(use_statistic_method_) {
                    vel_factor_ = std::min(1.0, hypot(rival_x_cov_, rival_y_cov_)/(cov_range_max_-cov_range_min_));
                    position_offset_ = std::max((rival_distance_ - safe_distance_), 0.0) * vel_factor_ * offset_vel_factor_weight_statistic_;
                    ExpandPointWithCircle(x, y, nav2_costmap_2d::MAX_NON_OBSTACLE, halted_inflation_radius_, halted_cost_scaling_factor_, rival_inscribed_radius_);
                    ExpandLine(x, y, nav2_costmap_2d::MAX_NON_OBSTACLE, moving_inflation_radius_, moving_cost_scaling_factor_, 0, 
                        max_extend_length_* vel_factor_ * expand_vel_factor_weight_statistic_ / std::max(rival_distance_, 1.0));
                } else {
                    vel_factor_ = std::min(1.0, hypot(v_from_localization_x_, v_from_localization_y_)/(vel_range_max_-vel_range_min_));
                    position_offset_ = std::max((rival_distance_ - safe_distance_), 0.0) * vel_factor_ * offset_vel_factor_weight_localization_;
                    ExpandPointWithCircle(x, y, nav2_costmap_2d::MAX_NON_OBSTACLE, halted_inflation_radius_, halted_cost_scaling_factor_, rival_inscribed_radius_);
                    ExpandLine(x, y, nav2_costmap_2d::MAX_NON_OBSTACLE, moving_inflation_radius_, moving_cost_scaling_factor_, 0, 
                        max_extend_length_ * vel_factor_ * expand_vel_factor_weight_localization_ / std::max(rival_distance_, 1.0));
                }
                break;
            
            case RivalState::UNKNOWN:
                ExpandPointWithCircle(x, y, nav2_costmap_2d::MAX_NON_OBSTACLE, unknown_inflation_radius_, unknown_cost_scaling_factor_, rival_inscribed_radius_);
                break;
            
            default:
                RCLCPP_WARN(
                    rclcpp::get_logger("RivalLayer"), 
                    "Unknown rival state");
                break;
        }
    }

    void RivalLayer::updateRadius() {
        auto node = node_.lock();
        // Update the inflation radius of the rival
                if(mode_param == 0) {
            node->get_parameter(name_ + "." + "halted_inflation_radius", halted_inflation_radius_);
            node->get_parameter(name_ + "." + "wandering_inflation_radius", wandering_inflation_radius_);
            node->get_parameter(name_ + "." + "moving_inflation_radius", moving_inflation_radius_);
            node->get_parameter(name_ + "." + "unknown_inflation_radius", unknown_inflation_radius_);
        } else if(mode_param == 1) {
            halted_inflation_radius_ = 0.02;
            wandering_inflation_radius_ = 0.02;
            moving_inflation_radius_ = 0.02;
            unknown_inflation_radius_ = 0.02;
        }
        // TODO: Refactor the implementation code for inflation_radius_ to avoid misleading the variable name
        halted_inflation_radius_ += rival_inscribed_radius_;
        wandering_inflation_radius_ += rival_inscribed_radius_;
        moving_inflation_radius_ += rival_inscribed_radius_;
        unknown_inflation_radius_ += rival_inscribed_radius_;
    }

    void RivalLayer::activate() {
        RCLCPP_INFO(
            rclcpp::get_logger("RivalLayer"), 
            "Activating RivalLayer");
    }
        
    void RivalLayer::deactivate() {
        RCLCPP_INFO(
            rclcpp::get_logger("RivalLayer"), 
            "Deactivating RivalLayer");
    }
            
    // Subscribe to the rival's pose
    void RivalLayer::rivalPoseCallback(const nav_msgs::msg::Odometry::SharedPtr rival_pose) {
        // Store the rival's pose
        rival_x_ = rival_pose->pose.pose.position.x;
        rival_y_ = rival_pose->pose.pose.position.y;
        v_from_localization_x_ = rival_pose->twist.twist.linear.x;
        v_from_localization_y_ = rival_pose->twist.twist.linear.y;
        rival_pose_received_ = true;
        
        // Pop the oldest pose if the queue is full
        if(rival_path_.isFull()) { 
            rival_x_sum_ -= rival_path_.get(0).first;
            rival_y_sum_ -= rival_path_.get(0).second;
            rival_x_sq_sum_ -= rival_path_.get(0).first * rival_path_.get(0).first;
            rival_y_sq_sum_ -= rival_path_.get(0).second * rival_path_.get(0).second;
            rival_xy_sum_ -= rival_path_.get(0).first * rival_path_.get(0).second;
        }

        // Store the rival's path and statistics
        rival_path_.push(std::make_pair(rival_x_, rival_y_));
        rival_x_sum_ += rival_x_;
        rival_y_sum_ += rival_y_;
        rival_x_sq_sum_ += rival_x_ * rival_x_;
        rival_y_sq_sum_ += rival_y_ * rival_y_;
        rival_xy_sum_ += rival_x_ * rival_y_;

        if(debug_mode_ == 3) {
            RCLCPP_INFO(
                rclcpp::get_logger("RivalLayer"), 
                "Statistics: x_mean=%f, y_mean=%f, x_cov=%f, y_cov=%f, R_sq=%f", rival_x_mean_, rival_y_mean_, rival_x_cov_, rival_y_cov_, R_sq_);
        }
    }
    void RivalLayer::rivalDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        rival_distance_ = msg->data;
        // RCLCPP_INFO(logger_, "rival_distance is : %f", msg->data);
    }

    void RivalLayer::PrintRivalState() {
        if(rival_state_ != rival_state_prev_) {
            if(debug_mode_ == 2) {
                RCLCPP_INFO(
                    rclcpp::get_logger("RivalLayer"), 
                    "Statistics: x_mean=%f, y_mean=%f, x_var=%f, y_var=%f, x_cov=%f, y_cov=%f, R_sq=%f", rival_x_mean_, rival_y_mean_, rival_x_var_, rival_y_var_, rival_x_cov_, rival_y_cov_, R_sq_);
            }

            switch (rival_state_) {
                case RivalState::HALTED:
                    RCLCPP_INFO(
                        rclcpp::get_logger("RivalLayer"), 
                        "Rival is HALTED");
                    break;

                case RivalState::WANDERING:
                    RCLCPP_INFO(
                        rclcpp::get_logger("RivalLayer"), 
                        "Rival is WANDERING");
                    break;
                
                case RivalState::MOVING:
                    RCLCPP_INFO(
                        rclcpp::get_logger("RivalLayer"), 
                        "Rival is MOVING");
                    break;

                case RivalState::UNKNOWN:
                    RCLCPP_INFO(
                        rclcpp::get_logger("RivalLayer"), 
                        "Rival state is UNKNOWN");
                    break;
                
                default:
                    RCLCPP_WARN(
                        rclcpp::get_logger("RivalLayer"), 
                        "Unknown rival state");
                    break;
            }
        }
        rival_state_prev_ = rival_state_;
    }

}   // namespace custom_path_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_path_costmap_plugin::RivalLayer, nav2_costmap_2d::Layer)
