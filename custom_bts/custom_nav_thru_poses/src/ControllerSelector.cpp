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
            controller_selector_pub_ = this->create_publisher<std_msgs::msg::String>("/controller_type", rclcpp::QoS(10).reliable().transient_local());
            controller_selector_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ControllerSelector::timer_callback, this));
        
            declare_parameter("Fast_controller", rclcpp::ParameterValue("FollowPath"));
            declare_parameter("Slow_controller", rclcpp::ParameterValue("Cautious"));

            this->get_parameter("Fast_controller", fast_controller);
            this->get_parameter("Slow_controller", slow_controller);
        }   

    private:
        void feedback_callback(nav2_msgs::action::NavigateThroughPoses::Feedback feedback) {
            if(feedback.number_of_poses_remaining <= 1) {
                controller_type_ = slow_controller_;
            } else {
                controller_type_ = fast_controller_;
            }

            if(controller_type_prev_ != controller_type_)    RCLCPP_INFO(this->get_logger(), "Controller type has switch to '%s'", controller_type_.c_str());

            controller_type_prev_ = controller_type_;
        }
        
        void timer_callback() {
            auto message = std_msgs::msg::String();

            // Hardcoded controller selection
            message.data = controller_type_;

            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

            controller_selector_pub_->publish(message);
        }

        // Navigate through poses feedback subscriber
        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage>::SharedPtr feedback_sub_;

        // Selected controller publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_selector_pub_;
        rclcpp::TimerBase::SharedPtr controller_selector_timer_;

        // Hardcoded controller selection
        std::string controller_type_;
        std::string controller_type_prev_ = "None";
        std::string fast_controller_, slow_controller_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerSelector>());
    rclcpp::shutdown();
    return 0;
}