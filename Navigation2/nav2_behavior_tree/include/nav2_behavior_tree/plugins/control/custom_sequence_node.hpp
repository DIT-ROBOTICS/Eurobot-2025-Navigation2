#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__CUSTOM_SEQUENCE_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__CUSTOM_SEQUENCE_NODE_HPP_
#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behavior_tree
{
class CustomSequence : public BT::ControlNode
{
    public:
        explicit CustomSequence(const std::string & name);
        CustomSequence(const std::string & name, const BT::NodeConfiguration & config);
        BT::NodeStatus tick() override;
        void halt() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("goalUpdated", false, "goal has changed")
            };
        }

    private:
        unsigned int current_child_idx_{0};
        unsigned int num_failed_children_{0};
        bool isGoalUpdated{false};
        void resetState();
};
}

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__CUSTOM_SEQUENCE_NODE_HPP_