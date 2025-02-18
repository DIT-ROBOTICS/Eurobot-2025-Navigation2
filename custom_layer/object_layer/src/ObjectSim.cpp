#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class ObjectSimPub : public rclcpp::Node {
    public:
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr column_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr board_pub_;
        rclcpp::TimerBase::SharedPtr column_timer_;
        rclcpp::TimerBase::SharedPtr board_timer_;
        ObjectSimPub() : Node("object_sim_pub") {
            column_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/column_pose_array", 100);
            board_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/board_pose_array", 100);
            column_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ObjectSimPub::column_timer_callback, this));
            board_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ObjectSimPub::board_timer_callback, this));
        }
    private:
        void column_timer_callback() {
            auto column_message = geometry_msgs::msg::PoseArray();
            column_message.header.stamp = this->now();
            column_message.header.frame_id = "column";
            for(int i = 0; i < 5; i++){
                auto column_pose = geometry_msgs::msg::Pose();
                column_pose.position.x = 0.5 + i * 0.5;
                column_pose.position.y = 0.5;
                column_message.poses.push_back(column_pose);
            }
            column_pub_->publish(column_message);
        }
        void board_timer_callback() {
            auto board_message = geometry_msgs::msg::PoseArray();
            board_message.header.stamp = this->now();
            board_message.header.frame_id = "board";
            for(int i = 0; i < 5; i++){
                auto board_pose = geometry_msgs::msg::Pose();
                board_pose.position.x = 0.5 + i * 0.5;
                board_pose.position.y = 1.5;
                board_message.poses.push_back(board_pose);
            }
            board_pub_->publish(board_message);
        }
           
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSimPub>());
    rclcpp::shutdown();
    return 0;
}