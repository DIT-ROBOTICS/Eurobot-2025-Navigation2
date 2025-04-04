#include "nav2_behaviors/plugins/shrink.hpp"

namespace nav2_behaviors
{
    Shrink::Shrink() : TimedBehavior<ShrinkAction>(){
    }

    Shrink::~Shrink(){
        setToOriginal();
    }

    void Shrink::onConfigure()
    {
        times = 0;
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error("Failed to lock node");
        }
        param_client = std::make_shared<rclcpp::AsyncParametersClient>(
            node, 
            "/global_costmap/global_costmap"
        );

        sub_costmap = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 
            rclcpp::QoS(10), 
            std::bind(&Shrink::costmapCallback, this, std::placeholders::_1)
        );
        getOriginalParam();
    }

    void Shrink::deactivate(){
        setToOriginal();
    }

    void Shrink::onActionCompletion(){
        setToOriginal();
    }

    Status Shrink::onRun(const std::shared_ptr<const ShrinkAction::Goal> command){
        unused_shrink = command->shrink_to;

        // Get the current robot pose (already done)
        nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);

        goalPose = robotPose;
        return Status::SUCCEEDED;
    }

    Status Shrink::onCycleUpdate(){
        times++;
        setToShrink();
        if(noCostInMiddle() && noCostAtGoal() && times > 20){
            // setToOriginal();
            times = 0;
            return Status::SUCCEEDED;
        }
        else if(times > 20){
            RCLCPP_ERROR(logger_, "shrink the inflation radius is not working");
            // setToOriginal();
            times = 0;
            return Status::FAILED;
        }
        else return Status::RUNNING;
    }

    void Shrink::getOriginalParam(){
        if(param_client->service_is_ready()){
            param_client->get_parameters({"inflation_layer.inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                original_inflation_radius = result[0].as_double();
            });
        }

        if(param_client->service_is_ready()){
            param_client->get_parameters({"rival_layer.halted_inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                original_rival_halted_radius = result[0].as_double();
            });
        }

        if(param_client->service_is_ready()){
            param_client->get_parameters({"rival_layer.wandering_inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                original_rival_wandering_radius = result[0].as_double();
            });
        }

        if(param_client->service_is_ready()){
            param_client->get_parameters({"rival_layer.moving_inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                original_rival_moving_radius = result[0].as_double();
            });
        }

        if(param_client->service_is_ready()){
            param_client->get_parameters({"rival_layer.unknown_inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                original_rival_unknown_radius = result[0].as_double();
            });
        }

        if(param_client->service_is_ready()){
            param_client->get_parameters({"object_layer.board_inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                original_object_board_radius = result[0].as_double();
            });
        }

        if(param_client->service_is_ready()){
            param_client->get_parameters({"object_layer.column_inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                original_object_column_radius = result[0].as_double();
            });
        }
    }

    void Shrink::setToOriginal(){
        changeInflationLayer(false);
        changeRivalLayer(false);
        changeObjectLayer(false);
    }

    void Shrink::setToShrink(){
        changeInflationLayer(true);
        changeRivalLayer(true);
        changeObjectLayer(true);
    }

    void Shrink::worldToMap(double wx, double wy, int & mx, int & my){
        mx = (int)((wx - costmap.info.origin.position.x) / costmap.info.resolution);
        my = (int)((wy - costmap.info.origin.position.y) / costmap.info.resolution);
    }

    double Shrink::getOneGridCost(double x, double y){
        int map_x, map_y;
        worldToMap(x, y, map_x, map_y);
        return costmap.data[map_y * costmap.info.width + map_x];
    }

    void Shrink::costmapCallback(const nav_msgs::msg::OccupancyGrid& msg){
        costmap = msg;
    }

    void Shrink::changeInflationLayer(bool doShrink){
        double radius = doShrink ? 0.1 : original_inflation_radius;
        if(param_client->service_is_ready()){
            param_client->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", radius)},[this, radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set inflation_radius successfully");
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set inflation_radius");
                }
            });
        }
        else{
            RCLCPP_ERROR(logger_, "Service is not ready");
        }
    }

    void Shrink::changeRivalLayer(bool doShrink){
        double halted_radius = doShrink ? 0.1 : original_rival_halted_radius;
        if(param_client->service_is_ready()){
            // rival_layer.halted_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("rival_layer.halted_inflation_radius", halted_radius)},[this, halted_radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set halted_inflation_radius successfully to %f", halted_radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set halted_inflation_radius to %f", halted_radius);
                }
            });
        }

        double wandering_radius = doShrink ? 0.1 : original_rival_wandering_radius;
        if(param_client->service_is_ready()){
            // rival_layer.wandering_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("rival_layer.wandering_inflation_radius", wandering_radius)},[this, wandering_radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set wandering_inflation_radius successfully to %f", wandering_radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set wandering_inflation_radius to %f", wandering_radius);
                }
            });
        }

        double moving_radius = doShrink ? 0.1 : original_rival_moving_radius;
        if(param_client->service_is_ready()){
            // rival_layer.moving_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("rival_layer.moving_inflation_radius", moving_radius)},[this, moving_radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set moving_inflation_radius successfully to %f", moving_radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set moving_inflation_radius to %f", moving_radius);
                }
            });
        }

        double unknown_radius = doShrink ? 0.1 : original_rival_unknown_radius;
        if(param_client->service_is_ready()){
            // rival_layer.unknown_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("rival_layer.unknown_inflation_radius", unknown_radius)},[this, unknown_radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set unknown_inflation_radius successfully to %f", unknown_radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set unknown_inflation_radius to %f", unknown_radius);
                }
            });
        }
    }

    void Shrink::changeObjectLayer(bool doShrink){
        double board_radius = doShrink ? 0.1 : original_object_board_radius;
        if(param_client->service_is_ready()){
            // object_layer.board_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("object_layer.board_inflation_radius", board_radius)},[this, board_radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set board_inflation_radius successfully to %f", board_radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set board_inflation_radius to %f", board_radius);
                }
            });
        }

        double column_radius = doShrink ? 0.1 : original_object_column_radius;
        if(param_client->service_is_ready()){
            // object_layer.column_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("object_layer.column_inflation_radius", column_radius)},[this, column_radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set column_inflation_radius successfully to %f", column_radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set column_inflation_radius to %f", column_radius);
                }
            });
        }
    }

    bool Shrink::noCostInMiddle(){
        int cost = getOneGridCost(robotPose.pose.position.x, robotPose.pose.position.y);
        if(cost > 50){
            RCLCPP_INFO(logger_, "obstacle detected at the center of the robot: the center %f, %f; the cost: %d", robotPose.pose.position.x, robotPose.pose.position.y, cost);
            return false;
        }
        else{
            return true;
        }
    }

    bool Shrink::noCostAtGoal(){
        int cost = getOneGridCost(goalPose.pose.position.x, goalPose.pose.position.y);
        if(cost > 50){
            RCLCPP_INFO(logger_, "obstacle detected at the center of the robot: the center %f, %f; the cost: %d", robotPose.pose.position.x, robotPose.pose.position.y, cost);
            return false;
        }
        else{
            return true;
        }
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Shrink, nav2_core::Behavior)