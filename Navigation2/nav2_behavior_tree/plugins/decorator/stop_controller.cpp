#include "nav2_behavior_tree/plugins/decorator/stop_controller.hpp"

namespace nav2_behavior_tree
{
    StopController::StopController(
        const std::string & name,
        const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf),
      stop_robot(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
        param_client = std::make_shared<rclcpp::AsyncParametersClient>(
            node_,
            "/behavior_server"
        );
        shrink_client = node_->create_client<std_srvs::srv::SetBool>(
            "/shrink/shrinkBack",
            rmw_qos_profile_services_default);
        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        stop_sub = node_->create_subscription<std_msgs::msg::Bool>(
            "/stopRobot",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&StopController::stopCallback, this, std::placeholders::_1),
            sub_option);
        shrink_timer_start = false;
        shrinkBack = false;
        shrink_timer = 0;
    }

    inline BT::NodeStatus StopController::tick()
    {
        callback_group_executor_.spin_some();

        getShrinkParam();
        if(checkShrinkBack()) doShrinkRequest();
        
        if (stop_robot)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub->publish(cmd_vel);
            RCLCPP_INFO(node_->get_logger(), "running in stop_controller");
            return BT::NodeStatus::FAILURE;
        }
        
        return child_node_->executeTick();
    }

    void StopController::stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        stop_robot = msg->data;
    }

    void StopController::getShrinkParam()
    {
        // if (param_client->service_is_ready())
        // {
        //     param_client->get_parameters({"shrinkBack"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future)
        //     {
        //         future.wait();
        //         auto result = future.get();
        //         shrinkBack = result[0].as_bool();
        //         RCLCPP_INFO(node_->get_logger(), "get shrinkBack: %d", shrinkBack);
        //     });
        // }
        // else
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Service is not ready");
        // }
        shrinkBack = node_->get_parameter("/behavior_server/shrinkBack", shrinkBack);
        if(shrinkBack) RCLCPP_INFO(node_->get_logger(), "shrinkBack: %d, is true", shrinkBack);
    }

    void StopController::doShrinkRequest(){
        if(!shrink_client->wait_for_service(std::chrono::milliseconds(100))){
            RCLCPP_ERROR(node_->get_logger(), "Service not available, waiting...");
        }
        else {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;
            auto result = shrink_client->async_send_request(request);
            if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS){
                if(result.get()->success) RCLCPP_INFO(node_->get_logger(), "Shrink request sent successfully");
                else RCLCPP_ERROR(node_->get_logger(), "Failed to send shrink request");
            }
            else RCLCPP_ERROR(node_->get_logger(), "Failed to send shrink request");
        }
    }
    
    bool StopController::checkShrinkBack(){
        // RCLCPP_INFO(node_->get_logger(), "shrink_timer: %d", shrink_timer);
        if(shrink_timer_start) shrink_timer++;
        else if(shrink_timer> 30) {
            shrink_timer_start = false;
            shrink_timer = 0;
            return true;
        }

        if(shrinkBack){
            if(!shrink_timer_start) {
                shrink_timer_start = true;
                return false;
            }
            else {
                if(shrink_timer >= 30){
                    shrink_timer_start = false;
                    shrink_timer = 0;
                    return true;
                }
            }
        }
        else return false;
        return false;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::StopController>("StopController");
}