#include "chrono"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

class ControllerSelector : public rclcpp::Node {
    public:
        ControllerSelector() : Node("controller_selector") {
            // Navigate through poses feedback subscriber
            feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>(
                "navigate_through_poses/_action/feedback",
                rclcpp::SystemDefaultsQoS(),
                [this](const nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage::SharedPtr msg) {
                    feedback_callback(msg->feedback);
                });

            // Selected controller publisher
            controller_selector_pub_ = this->create_publisher<std_msgs::msg::String>("/controller_type_thru", rclcpp::QoS(10).reliable().transient_local());
        
            declare_parameter("Fast_controller", rclcpp::ParameterValue("Fast"));
            declare_parameter("Slow_controller", rclcpp::ParameterValue("Slow"));

            this->get_parameter("Fast_controller", fast_controller_);
            this->get_parameter("Slow_controller", slow_controller_);
        }   

    private:
        void feedback_callback(nav2_msgs::action::NavigateThroughPoses::Feedback feedback) {
            // RCLCPP_INFO(this->get_logger(), "Feedback received: %d poses remaining", feedback.number_of_poses_remaining);
            // TODO: Check if the feedback is stable
            if(feedback.number_of_poses_remaining <= 1) {
                controller_type_.data = slow_controller_;
            } else {
                controller_type_.data = fast_controller_;
            }
            
            if(controller_type_prev_ != controller_type_.data) {
                controller_selector_pub_->publish(controller_type_);
                // RCLCPP_INFO(this->get_logger(), "Controller type has switch to '%s'", controller_type_.data.c_str());
            }

            controller_type_prev_ = controller_type_.data;
        }

        // Navigate through poses feedback subscriber
        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr feedback_sub_;

        // Selected controller publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_selector_pub_;

        // Hardcoded controller selection
        std_msgs::msg::String controller_type_;
        std::string controller_type_prev_ = "None";
        std::string fast_controller_, slow_controller_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerSelector>());
    rclcpp::shutdown();
    return 0;
}