#include "nav2_behaviors/plugins/shrink.hpp"

namespace nav2_behaviors
{
    Shrink::Shrink() 
    : TimedBehavior<ShrinkAction>(),
      inflation_radius(0.0) 
    {
    }

    Shrink::~Shrink() 
    {
        if(param_client->service_is_ready()){
            param_client->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", 0.1)},[this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
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

    void Shrink::changeInflationLayer(double radius){
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

    void Shrink::changeRivalLayer(double radius){
        if(param_client->service_is_ready()){
            // rival_layer.halted_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("rival_layer.halted_inflation_radius", radius)},[this, radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set halted_inflation_radius successfully to %f", radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set halted_inflation_radius to %f", radius);
                }
            });

            // rival_layer.wandering_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("rival_layer.wandering_inflation_radius", radius)},[this, radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set wandering_inflation_radius successfully to %f", radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set wandering_inflation_radius to %f", radius);
                }
            });

            // rival_layer.moving_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("rival_layer.moving_inflation_radius", radius)},[this, radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set moving_inflation_radius successfully to %f", radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set moving_inflation_radius to %f", radius);
                }
            });

            // rival_layer.unknow_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("rival_layer.unknow_inflation_radius", radius)},[this, radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set unknow_inflation_radius successfully to %f", radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set unknow_inflation_radius to %f", radius);
                }
            });
        }
        else{
            RCLCPP_ERROR(logger_, "Service is not ready");
        }
    }

    void Shrink::changeObjectLayer(double radius){
        if(param_client->service_is_ready()){
            // object_layer.board_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("object_layer.board_inflation_radius", radius)},[this, radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set board_inflation_radius successfully to %f", radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set board_inflation_radius to %f", radius);
                }
            });

            // object_layer.column_inflation_radius
            param_client->set_parameters({rclcpp::Parameter("object_layer.column_inflation_radius", radius)},[this, radius](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future){
                future.wait();
                auto result = future.get();
                if(result[0].successful){
                    RCLCPP_INFO(logger_, "Set column_inflation_radius successfully to %f", radius);
                }
                else{
                    RCLCPP_ERROR(logger_, "Failed to set column_inflation_radius to %f", radius);
                }
            });
        }
        else{
            RCLCPP_ERROR(logger_, "Service is not ready");
        }
    }

    void Shrink::setInflationRadius(double radius){
        inflation_radius = radius;
        // inflation_layer
        changeInflationLayer(inflation_radius);
        // rival_layer
        changeRivalLayer(inflation_radius);
        // object_layer.board_inflation_radius
        changeObjectLayer(inflation_radius);
    }

    bool Shrink::isWork(){
        int cost = getOneGridCost(robotPose.pose.position.x, robotPose.pose.position.y);
        if(cost > 50){
            RCLCPP_INFO(logger_, "obstacle detected at the center of the robot: the center %f, %f; the cost: %d", robotPose.pose.position.x, robotPose.pose.position.y, cost);
            return false;
        }
        else{
            RCLCPP_INFO(logger_, "no obstacle detected at the center of the robot: the center %f, %f; the cost: %d", robotPose.pose.position.x, robotPose.pose.position.y, cost);
            return true;
        }
    }

    Status Shrink::onRun(const std::shared_ptr<const ShrinkAction::Goal> command){
        inflation_radius = command->shrink_to;
        nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);
        setInflationRadius(inflation_radius);
        return Status::SUCCEEDED;
    }

    Status Shrink::onCycleUpdate(){
        times++;
        if(isWork()){
            setInflationRadius(0.1);
            return Status::SUCCEEDED;
        }
        else if(times > 10){
            RCLCPP_ERROR(logger_, "shrink the inflation radius is not working");
            setInflationRadius(0.1);
            times = 0;
            return Status::FAILED;
        }
        else return Status::RUNNING;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Shrink, nav2_core::Behavior)