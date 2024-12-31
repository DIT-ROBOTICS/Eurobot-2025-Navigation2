#include "chrono"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class RivalSimPub : public rclcpp::Node {
    public:
        RivalSimPub() : Node("rival_sim_pub") {
            pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/rival_pose", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RivalSimPub::timer_callback, this));
        }   

    private:
        void timer_callback() {
            auto message = geometry_msgs::msg::PoseWithCovarianceStamped();

            // header
            message.header.stamp = this->now();
            message.header.frame_id = "map";

            // position
            message.pose.pose.position.x += move_x_;
            message.pose.pose.position.y += move_y_;
            message.pose.pose.position.z = 0.0;

            move_x_ += 0.1 * toggle_x_;
            move_y_ += 0.1 * toggle_y_;

            if (move_x_ > 2.5 || move_x_ < 0.5) {
                toggle_x_ *= -1;
            }
            if (move_y_ > 1.5 || move_y_ < 0.5) {
                toggle_y_ *= -1;
            }

            RCLCPP_INFO(this->get_logger(), "Publishing: x=%f, y=%f, z=%f\n", message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z);
            pub_->publish(message);
        }

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        double move_x_ = 1.5;
        double move_y_ = 1.0;
        int toggle_x_ = 1;
        int toggle_y_ = 1;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RivalSimPub>());
    rclcpp::shutdown();
    return 0;
}
