#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <random>
#include <vector>

class ObjectSimPub : public rclcpp::Node {
    public:
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr column_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr board_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr overturn_pub_; // New publisher
        rclcpp::TimerBase::SharedPtr column_timer_;
        rclcpp::TimerBase::SharedPtr board_timer_;
        rclcpp::TimerBase::SharedPtr overturn_timer_; // New timer
        int change_position_column = 50;
        int change_position_board = 90;
        geometry_msgs::msg::PoseArray column_message;
        geometry_msgs::msg::PoseArray board_message;
        geometry_msgs::msg::PoseArray overturn_message; // New message
        int mode = 2;
        std::vector<double> column_pos_x {1.5, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 2.9, 2.9, 2.9, 2.9, 2.9, 2.9, 2.9, 2.9, 1.35, 1.3, 1.25, 1.2, 1.65, 1.7, 1.75, 1.8, 0.85, 0.8, 0.75, 0.7, 2.15, 2.2, 2.25, 2.3, 0.85, 0.8, 0.75, 0.7, 2.15, 2.2, 2.25, 2.3};
        std::vector<double> column_pos_y {1, 0.75, 0.7, 0.65, 0.6, 1.75, 1.7, 1.65, 1.6, 0.75, 0.7, 0.65, 0.6, 1.75, 1.7, 1.65, 1.6, 1, 1, 1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8};
        std::vector<double> custom_column_pos_x {1.5};
        std::vector<double> custom_column_pos_y {1};
        std::vector<double> board_pos_x {1.8};
        std::vector<double> board_pos_y {1};
        std::vector<double> board_orientation { /* fill with desired values */ };
        
        // New vectors for overturn positions
        std::vector<double> overturn_pos_x {1.0, 2.0};  // Example values
        std::vector<double> overturn_pos_y {1.0, 1.5};  // Example values
        std::vector<double> overturn_orientation {0.0, M_PI/2};  // Example orientations
        
        ObjectSimPub() : Node("object_sim_pub") {
            column_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/global_center_poses/column", 100);
            board_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/global_center_poses/platform", 100);
            overturn_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/global_center_poses/overturn", 100); // New publisher
            
            column_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ObjectSimPub::column_timer_callback, this));
            board_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ObjectSimPub::board_timer_callback, this));
            overturn_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ObjectSimPub::overturn_timer_callback, this)); // New timer
        }
    private:
        void column_timer_callback() {
            if(mode == 0){
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
                else column_pub_->publish(column_message);
                change_position_column--;
            }
            else if(mode == 1){
                column_message = geometry_msgs::msg::PoseArray();
                column_message.header.stamp = this->now();
                column_message.header.frame_id = "column";
                for(int i = 0; i < column_pos_x.size(); i++){
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = column_pos_x[i];
                    pose.position.y = column_pos_y[i];
                    pose.orientation.z = std::sin(0.0 / 2.0);
                    pose.orientation.w = std::cos(0.0 / 2.0);
                    column_message.poses.push_back(pose);
                }
                column_pub_->publish(column_message);
            }
            else if(mode == 2){
                column_message = geometry_msgs::msg::PoseArray();
                column_message.header.stamp = this->now();
                column_message.header.frame_id = "column";
                for(int i = 0; i < custom_column_pos_x.size(); i++){
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = custom_column_pos_x[i];
                    pose.position.y = custom_column_pos_y[i];
                    pose.orientation.z = std::sin(0.0 / 2.0);
                    pose.orientation.w = std::cos(0.0 / 2.0);
                    column_message.poses.push_back(pose);
                }
                column_pub_->publish(column_message);
            }
        }
        void board_timer_callback() {
            if(mode == 0){
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
                else board_pub_->publish(board_message);
                change_position_board--;
            }
            else if(mode == 1){
                return;
            }
            else if(mode == 2) {
                board_message = geometry_msgs::msg::PoseArray();
                board_message.header.stamp = this->now();
                board_message.header.frame_id = "board";
                for(int i = 0; i < board_pos_x.size(); i++){
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = board_pos_x[i];
                    pose.position.y = board_pos_y[i];
                    pose.orientation.z = std::sin(0.0 / 2.0);
                    pose.orientation.w = std::cos(0.0 / 2.0);
                    board_message.poses.push_back(pose);
                }
                board_pub_->publish(board_message);
            }
        }

        // New callback for overturn timer
        void overturn_timer_callback() {
            if(mode == 0){
                // Random mode implementation similar to board_timer_callback
                // (Implementation omitted for brevity)
            }
            else if(mode == 1){
                // Not implemented for mode 1
                return;
            }
            else if(mode == 2){
                overturn_message = geometry_msgs::msg::PoseArray();
                overturn_message.header.stamp = this->now();
                overturn_message.header.frame_id = "map";
                for(size_t i = 0; i < overturn_pos_x.size() && i < overturn_pos_y.size(); i++){
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = overturn_pos_x[i];
                    pose.position.y = overturn_pos_y[i];
                    
                    // Use orientation if available, otherwise default to 0
                    double yaw = (i < overturn_orientation.size()) ? overturn_orientation[i] : 0.0;
                    pose.orientation.x = 0.0;  // Used for object type in your implementation
                    pose.orientation.y = 0.0;
                    pose.orientation.z = std::sin(yaw / 2.0);
                    pose.orientation.w = std::cos(yaw / 2.0);
                    
                    overturn_message.poses.push_back(pose);
                }
                overturn_pub_->publish(overturn_message);
            }
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