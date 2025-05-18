#include "chrono"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class RivalSimPub : public rclcpp::Node {
    public:
        RivalSimPub() : Node("rival_sim_pub") {
            rival_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/rival/final_pose", 100);
            rival_timer_ = this->create_wall_timer(std::chrono::milliseconds(time_span_), std::bind(&RivalSimPub::timer_callback, this));

            // Get the rival mode -> 0: Halted, 1: Wandering, 2: Moving, 3: Moving with noise
            declare_parameter("Rival_mode", rclcpp::ParameterValue(2));
            this->get_parameter("Rival_mode", rival_mode_);
        }   

    private:
        void timer_callback() {
            trigger_once_cnt_--;

            if(trigger_once_cnt_ > 0) {
                auto message = nav_msgs::msg::Odometry();

                // header
                message.header.stamp = this->now();
                message.header.frame_id = "map";

                // position
                message.pose.pose.position.x = 2.8;
                message.pose.pose.position.y = 1.0;

                rival_pub_->publish(message);
            } else {  
                auto message = nav_msgs::msg::Odometry();

                // header
                message.header.stamp = this->now();
                message.header.frame_id = "map";

                // position
                // Hardcoded moving rival
                if (pause_) {
                    cooldown_++;
                    
                    message.pose.pose.position.x = pause_pose_.pose.pose.position.x;
                    message.pose.pose.position.y = pause_pose_.pose.pose.position.y;
                    message.twist.twist.linear.x = 0.0;
                    message.twist.twist.linear.y = 0.0;

                    if (cooldown_ > 50) {
                        pause_ = false;
                        cooldown_ = 0;
                    }

                } else {
                    message.pose.pose.position.x += move_x_;
                    message.pose.pose.position.y += move_y_;
                    message.twist.twist.linear.x = 10.0/float(time_span_);
                    message.twist.twist.linear.y = 10.0/float(time_span_);

                    if(rival_mode_ == 3) {
                        move_x_ += (0.01+float(rand()%5-2)/400.0) * toggle_x_;
                        move_y_ += (0.01+float(rand()%5-2)/400.0) * toggle_y_;
                    }

                    if(rival_mode_ == 2) {
                        move_x_ += 0.01 * toggle_x_;
                        move_y_ += 0.01 * toggle_y_;
                    }

                    if (move_x_ > 2.5 || move_x_ < 0.5) {
                        toggle_x_ *= -1;
                        pause_ = true;
                        pause_pose_ = message;
                    }
                    if (move_y_ > 1.5 || move_y_ < 0.5) {
                        toggle_y_ *= -1;
                        pause_ = true;
                        pause_pose_ = message;
                    }
                }

                // Hardcoded wandering rival
                if(rival_mode_ == 1) {
                    message.pose.pose.position.x = float(rand()%51-25)/200.0 + 1.5;
                    message.pose.pose.position.y = float(rand()%51-25)/200.0 + 1.0;
                }

                // Hardcoded halted rival
                if(rival_mode_ == 0) {
                    message.pose.pose.position.x = float(rand()%5-2)/200.0 + 2.8;
                    message.pose.pose.position.y = float(rand()%5-2)/200.0 + 1.0;
                }

                // RCLCPP_INFO(this->get_logger(), "Publishing: x=%f, y=%f\n", message.pose.pose.position.x, message.pose.pose.position.y);
                rival_pub_->publish(message);
            }
        }

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rival_pub_;
        rclcpp::TimerBase::SharedPtr rival_timer_;
        int rival_mode_ = 0;
        double move_x_ = 1.5;
        double move_y_ = 1.0;
        int toggle_x_ = 1;
        int toggle_y_ = 1;
        bool pause_ = false;
        int cooldown_ = 0;
        int trigger_once_cnt_ = 50;
        int time_span_ = 50;
        nav_msgs::msg::Odometry pause_pose_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RivalSimPub>());
    rclcpp::shutdown();
    return 0;
}
