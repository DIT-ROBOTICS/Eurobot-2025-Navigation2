#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <random>

class ObjectSimPub : public rclcpp::Node {
    public:
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr column_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr board_pub_;
        rclcpp::TimerBase::SharedPtr column_timer_;
        rclcpp::TimerBase::SharedPtr board_timer_;
        int change_position_column = 50;
        int change_position_board = 90;
        geometry_msgs::msg::PoseArray column_message;
        geometry_msgs::msg::PoseArray board_message;
        ObjectSimPub() : Node("object_sim_pub") {
            column_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/column_pose_array", 100);
            board_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/board_pose_array", 100);
            column_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ObjectSimPub::column_timer_callback, this));
            board_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ObjectSimPub::board_timer_callback, this));
        }
    private:
        void column_timer_callback() {
            if(change_position_column == 0){
                column_message = geometry_msgs::msg::PoseArray();
                column_message.header.stamp = this->now();
                column_message.header.frame_id = "column";
                for(int i = 0; i < 5; i++){
                    auto column_pose = generate_random_pose();
                    column_message.poses.push_back(column_pose);
                }
                column_pub_->publish(column_message);
                change_position_column = 50;
            }
            change_position_column--;
        }
        void board_timer_callback() {
            if(change_position_board == 0){
                board_message = geometry_msgs::msg::PoseArray();
                board_message.header.stamp = this->now();
                board_message.header.frame_id = "board";
                for(int i = 0; i < 5; i++){
                    auto board_pose = generate_random_pose();
                    board_message.poses.push_back(board_pose);
                }
                board_pub_->publish(board_message);
                change_position_board = 90;
            }
            change_position_board--;
        }

        geometry_msgs::msg::Pose generate_random_pose(){
            geometry_msgs::msg::Pose pose;

            // Random engines and distributions for positions and yaw
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_real_distribution<double> dist_x(0.0, 3.0);
            std::uniform_real_distribution<double> dist_y(0.0, 2.0);
            std::uniform_real_distribution<double> dist_yaw(0.0, 2*M_PI);
            
            // Generate random position
            pose.position.x = dist_x(gen);
            pose.position.y = dist_y(gen);
            
            // Generate random orientation using a random yaw
            double yaw = dist_yaw(gen);
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = std::sin(yaw / 2.0);
            pose.orientation.w = std::cos(yaw / 2.0);
            
            return pose;
        }
           
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSimPub>());
    rclcpp::shutdown();
    return 0;
}