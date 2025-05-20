#include "nav2_behavior_tree/plugins/decorator/setup_updater.hpp"

namespace nav2_behavior_tree
{
    SetupUpdater::SetupUpdater(
        const std::string & name,
        const BT::NodeConfiguration & conf)
        : BT::DecoratorNode(name, conf)
    {
        // Retrieve node from the blackboard
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        // Create client to shrink's service
        shrink_client_ = node_->create_client<std_srvs::srv::SetBool>(
            "/shrink/doneShrink",
            rmw_qos_profile_services_default);
    }

    void SetupUpdater::requestShrinkBack()
    {
        if (!shrink_client_->wait_for_service(std::chrono::milliseconds(100))) {
            RCLCPP_ERROR(node_->get_logger(), "Shrink service not available, waiting...");
        } else {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;  // Instruct shrink to call setToOriginal()
            auto result = shrink_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = result.get();
                if (response->success)
                    RCLCPP_INFO(node_->get_logger(), "Shrink request sent successfully");
                else
                    RCLCPP_ERROR(node_->get_logger(), "Shrink service call failed");
            }
            else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to send shrink service call");
            }
        }
    }

    bool SetupUpdater::goalUpdated(){
        std::vector<geometry_msgs::msg::PoseStamped> current_goals;
        config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
        geometry_msgs::msg::PoseStamped current_goal;
        config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

        if (goal_ != current_goal || goals_ != current_goals) {
            goal_ = current_goal;
            goals_ = current_goals;
            return true;
        }

        return false;
    }

    inline BT::NodeStatus SetupUpdater::tick()
    {
        if(status() == BT::NodeStatus::IDLE) {
            config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
            config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
        }

        if(goalUpdated()) requestShrinkBack();

        return child_node_->executeTick();
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::SetupUpdater>("SetupUpdater");
}