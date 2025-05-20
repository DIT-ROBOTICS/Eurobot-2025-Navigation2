#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SETUP_UPDATER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SETUP_UPDATER_HPP_

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{
    class SetupUpdater : public BT::DecoratorNode
    {
        public:
            SetupUpdater(const std::string & name, const BT::NodeConfiguration & conf);
            static BT::PortsList providedPorts()
            {
                return { BT::InputPort<bool>("setup") };
            }
            BT::NodeStatus tick() override;
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr shrink_client_;
            geometry_msgs::msg::PoseStamped goal_;
            std::vector<geometry_msgs::msg::PoseStamped> goals_;
            void requestShrinkBack();
            bool goalUpdated();
    };
}


#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__SETUP_UPDATER_HPP_