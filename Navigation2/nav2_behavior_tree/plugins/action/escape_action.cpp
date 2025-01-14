#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/escape_action.hpp"

namespace nav2_behavior_tree
{
    EscapeAction::EscapeAction(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf)
    : BtActionNode<nav2_msgs::action::Escape>(xml_tag_name, action_name, conf)
    {
        double dist;
        getInput("run_dist", dist);
        double time_allowance;
        getInput("time_allowance", time_allowance);
        // goal_.target_distance = dist;
        goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
        getInput("is_recovery", is_recovery_);
        if(is_recovery_){
            RCLCPP_WARN(node_->get_logger(), "start escape_action node");
        }
    }

    void EscapeAction::on_tick()
    {   
        int i = 0;
        if (is_recovery_) {
            RCLCPP_WARN(node_->get_logger(), "Run escape_action node %d", i);
            i++;
        }
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<nav2_behavior_tree::EscapeAction>(name, "escape", config);
        };

    factory.registerBuilder<nav2_behavior_tree::EscapeAction>("Escape", builder);
}