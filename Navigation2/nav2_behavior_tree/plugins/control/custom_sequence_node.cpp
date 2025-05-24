#include "nav2_behavior_tree/plugins/control/custom_sequence_node.hpp"

namespace nav2_behavior_tree
{
    CustomSequence::CustomSequence(const std::string & name)
    : BT::ControlNode(name, {}){
        isGoalUpdated = false;
    }

    CustomSequence::CustomSequence(const std::string & name, const BT::NodeConfiguration & config)
    : BT::ControlNode(name, config){
        isGoalUpdated = false;
    }

    BT::NodeStatus CustomSequence::tick(){
        const auto num_children = children_nodes_.size();

        // Read the input port value each tick
        getInput("goalUpdated", isGoalUpdated);
        
        // Log the input value
        RCLCPP_INFO(
            rclcpp::get_logger("CustomSequence"), 
            "\033[1;34mGoal updated input: %s\033[0m", 
            isGoalUpdated ? "true" : "false");

        setStatus(BT::NodeStatus::RUNNING);
        if(isGoalUpdated) {
            RCLCPP_INFO(
                rclcpp::get_logger("CustomSequence"), 
                "\033[1;32mResetting state due to goal update\033[0m");
            resetState();
        }
        
        while (num_failed_children_ < num_children) {
            TreeNode * child_node = children_nodes_[current_child_idx_];
            const BT::NodeStatus child_status = child_node->executeTick();

            switch (child_status) {
            case BT::NodeStatus::SUCCESS:
                // Wrap around to the first child
                if (++current_child_idx_ >= num_children) {
                    current_child_idx_ = 0;
                }
                num_failed_children_ = 0;
                ControlNode::haltChildren();
                return BT::NodeStatus::SUCCESS;
            case BT::NodeStatus::FAILURE:
                if (++current_child_idx_ >= num_children) {
                    current_child_idx_ = 0;
                }
                num_failed_children_++;
                break;
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;
            default:
                throw BT::LogicError("Invalid status return from BT node");
            }
        }

        halt();
        return BT::NodeStatus::FAILURE;
    }

    void CustomSequence::halt(){
        ControlNode::halt();
        current_child_idx_ = 0;
        num_failed_children_ = 0;
    }

    void CustomSequence::resetState(){
        current_child_idx_ = 0;
        num_failed_children_ = 0;
        ControlNode::haltChildren();
    }
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::CustomSequence>("CustomSequence");
}