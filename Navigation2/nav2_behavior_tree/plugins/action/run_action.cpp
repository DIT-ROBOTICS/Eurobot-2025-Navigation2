#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/run_action.hpp"

namespace nav2_behavior_tree
{
    RunAction::RunAction(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf)
    : BtActionNode<nav2_msgs::action::Run>(xml_tag_name, action_name, conf)
    {
        double dist;
        getInput("run_dist", dist);
        double time_allowance;
        getInput("time_allowance", time_allowance);
        // goal_.target_distance = dist;
        goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
        getInput("is_recovery", is_recovery_);
        if(is_recovery_){
            RCLCPP_WARN(node_->get_logger(), "start run_action node");
        }
    }

    void RunAction::on_tick()
    {   
        int i = 0;
        if (is_recovery_) {
            RCLCPP_WARN(node_->get_logger(), "Run run_action node %d", i);
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
            return std::make_unique<nav2_behavior_tree::RunAction>(name, "run", config);
        };

    factory.registerBuilder<nav2_behavior_tree::RunAction>("Run", builder);
}