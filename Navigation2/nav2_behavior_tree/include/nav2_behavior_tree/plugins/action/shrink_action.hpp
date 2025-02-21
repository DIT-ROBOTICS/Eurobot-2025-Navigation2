#ifndef NAV2_BEHAVIOR_TREE_PLUGINS_ACTION_SHRINK_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE_PLUGINS_ACTION_SHRINK_ACTION_HPP_

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/shrink.hpp"

namespace nav2_behavior_tree
{
    class ShrinkAction : public BtActionNode<nav2_msgs::action::Shrink>
    {
        public:
            ShrinkAction(
                const std::string & xml_tag_name,
                const std::string & action_name,
                const BT::NodeConfiguration & conf);

            void on_tick() override;

            static BT::PortsList providedPorts()
            {
                return providedBasicPorts(
                    {
                        BT::InputPort<double>("shrink_to", 0.05, "shrink the inflation radius to this value")
                    });
            }
    };
}

#endif  // NAV2_BEHAVIOR_TREE_PLUGINS_ACTION_SHRINK_ACTION_HPP_
