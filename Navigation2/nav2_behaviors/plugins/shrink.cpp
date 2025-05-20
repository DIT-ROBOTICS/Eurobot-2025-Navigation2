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

        radius_param_client = std::make_shared<rclcpp::AsyncParametersClient>(
            node, 
            "/global_costmap/global_costmap"
        );

        sub_costmap = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 
            rclcpp::QoS(10), 
            std::bind(&Shrink::costmapCallback, this, std::placeholders::_1)
        );

        goal_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 
            rclcpp::SystemDefaultsQoS(), 
            std::bind(&Shrink::goalPoseCallback, this, std::placeholders::_1)
        );
        shrinkBack = false;
        node->get_parameter("costmap_tolerance", costmap_tolerance);

        shrinkCheck_srv = node->create_service<std_srvs::srv::SetBool>(
            "/shrink/doneShrink",
            std::bind(&Shrink::handleShrinkCheck, this, std::placeholders::_1, std::placeholders::_2)
        );

        setMode_rival_client = node->create_client<std_srvs::srv::SetBool>(
            "/rival_layer/set_mode", 
            rmw_qos_profile_services_default
        );

        setMode_object_client = node->create_client<std_srvs::srv::SetBool>(
            "/object_layer/set_mode", 
            rmw_qos_profile_services_default
        );

        setMode_inflation_client = node->create_client<std_srvs::srv::SetBool>(
            "/inflation_layer/set_mode", 
            rmw_qos_profile_services_default
        );

    }

    Status Shrink::onRun(const std::shared_ptr<const ShrinkAction::Goal> command){
        unused_shrink = command->shrink_to;

        // Get the current robot pose (already done)
        nav2_util::getCurrentPose(robotPose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);

        return Status::SUCCEEDED;
    }

    Status Shrink::onCycleUpdate(){
        times++;
        if(times == 3){
            setToShrink();
        }

        if(noCostInMiddle() && noCostAtGoal() && times > 20){
            times = 0;
            shrinkBack = true;
            return Status::SUCCEEDED;
        }
        else if(times > 20){
            RCLCPP_ERROR(logger_, "shrink the inflation radius is not working");
            times = 0;
            shrinkBack = true;
            return Status::FAILED;
        }
        else return Status::RUNNING;
    }

    void Shrink::handleShrinkCheck(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if(request->data){
            response->success = true;
            response->message = "getting message from the service";
            setToOriginal();
        }
    }

    void Shrink::getOriginalParam(){
        if(radius_param_client->service_is_ready()){
            radius_param_client->get_parameters({"inflation_layer.inflation_radius"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future){
                future.wait();
                auto result = future.get();
                original_inflation_radius = result[0].as_double();
            });
        }
    }

    void Shrink::setToOriginal(){
        RCLCPP_INFO(logger_, "set the inflation radius to original");
        changeInflationLayer(false);
        changeRivalLayer(false);
        changeObjectLayer(false);
    }

    void Shrink::setToShrink(){
        RCLCPP_INFO(logger_, "shrink the inflation radius");
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

    void Shrink::goalPoseCallback(const geometry_msgs::msg::PoseStamped& msg){
        goalPose = msg;
    }

    void Shrink::changeInflationLayer(bool doShrink) {
        if (!setMode_inflation_client->service_is_ready()) {
            RCLCPP_ERROR(logger_, "Service is not ready for inflation layer");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = doShrink;

        setMode_inflation_client->async_send_request(
            request,
            [this, doShrink](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(logger_, "Inflation layer mode set to %s", doShrink ? "shrink" : "original");
                    } else {
                        RCLCPP_ERROR(logger_, "Failed to set inflation layer mode: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger_, "Exception in inflation layer callback: %s", e.what());
                }
            });

        RCLCPP_INFO(logger_, "Requested inflation layer update");
    }

    void Shrink::changeRivalLayer(bool doShrink) {
        if (!setMode_rival_client->service_is_ready()) {
            RCLCPP_ERROR(logger_, "Service is not ready for rival layer");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = doShrink;
        
        setMode_rival_client->async_send_request(
            request,
            [this, doShrink](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(logger_, "Rival layer mode set to %s", doShrink ? "shrink" : "original");
                    } else {
                        RCLCPP_ERROR(logger_, "Failed to set rival layer mode: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger_, "Exception in rival layer callback: %s", e.what());
                }
            });
        
        RCLCPP_INFO(logger_, "Requested rival layer update asynchronously");
    }

    void Shrink::changeObjectLayer(bool doShrink) {
        if (!setMode_object_client->service_is_ready()) {
            RCLCPP_ERROR(logger_, "Service is not ready for object layer");
            return;
        }
        
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = doShrink;
        
        // Use a callback-based approach instead of spin_until_future_complete
        setMode_object_client->async_send_request(
            request,
            [this, doShrink](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(logger_, "Object layer mode set to %s", doShrink ? "shrink" : "original");
                    } else {
                        RCLCPP_ERROR(logger_, "Failed to set object layer mode: %s", response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger_, "Exception in object layer callback: %s", e.what());
                }
            });
        
        RCLCPP_INFO(logger_, "Requested object layer update asynchronously");
    }

    bool Shrink::noCostInMiddle(){
        int cost = getOneGridCost(robotPose.pose.position.x, robotPose.pose.position.y);
        if(cost > costmap_tolerance){
            return false;
        }
        else{
            return true;
        }
    }

    bool Shrink::noCostAtGoal(){
        int cost = getOneGridCost(goalPose.pose.position.x, goalPose.pose.position.y);
        if(cost > costmap_tolerance){
            return false;
        }
        else{
            return true;
        }
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Shrink, nav2_core::Behavior)