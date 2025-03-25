#include <memory>
#include "nav2_behavior_tree/plugins/action/shrink_action.hpp"

namespace nav2_behavior_tree
{
    ShrinkAction::ShrinkAction(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf)
    : BtActionNode<nav2_msgs::action::Shrink>(xml_tag_name, action_name, conf)
    {
        double shrink_to;
        getInput("shrink_to", shrink_to);
        RCLCPP_INFO(node_->get_logger(), "ShrinkAction: Shrinking the inflation radius to %f", shrink_to);
        goal_.shrink_to = shrink_to;
        RCLCPP_INFO(node_->get_logger(), "ShrinkAction: goal_.shrink_to : %f", goal_.shrink_to);
    }

    void ShrinkAction::on_tick()
    {
        RCLCPP_INFO(node_->get_logger(), "ShrinkAction: Shrinking the inflation radius to %f", goal_.shrink_to);
        increment_recovery_count();
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<nav2_behavior_tree::ShrinkAction>(name, "shrink", config);
        };

    factory.registerBuilder<nav2_behavior_tree::ShrinkAction>("Shrink", builder);
}