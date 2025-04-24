#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class NavTypeSelector
{
public:
  NavTypeSelector(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);
  ~NavTypeSelector() = default;

  void setType(std::string const & mode, char & offset_direction, geometry_msgs::msg::PoseStamped & original_staging_pose, double const & offset);

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_selector_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_checker_selector_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_function_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dock_controller_selector_pub_;

  std_msgs::msg::String controller_selector_msg_;
  std_msgs::msg::String goal_checker_selector_msg_;
  std_msgs::msg::String controller_function_msg_;
  std_msgs::msg::String dock_controller_selector_msg_;
};