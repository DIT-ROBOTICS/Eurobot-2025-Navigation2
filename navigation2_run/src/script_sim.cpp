#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <opennav_docking_msgs/action/dock_robot.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

std::string get_timestamped_filename() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm * tm = std::localtime(&now_time);

    std::ostringstream oss;
    oss << std::put_time(tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

class ScriptSim : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using DockRobot = opennav_docking_msgs::action::DockRobot;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using GoalHandleDock = rclcpp_action::ClientGoalHandle<DockRobot>;

    ScriptSim()
    : Node("script_sim")
    {
        // Create action clients
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
        dock_robot_client_ = rclcpp_action::create_client<DockRobot>(this, "/dock_robot");

        // Create publisher for controller & goal checker selector
        controller_selector_pub_ = this->create_publisher<std_msgs::msg::String>("/controller_type", rclcpp::QoS(10).reliable().transient_local());
        goal_checker_selector_pub_ = this->create_publisher<std_msgs::msg::String>("/goal_checker_type", rclcpp::QoS(10).reliable().transient_local());

        // Create subscriber for localization data
        final_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/final_pose", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                final_pose_data_ = *msg;
            });
        
        lidar_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/lidar_pose", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                lidar_pose_data_ = *msg;
            });

        beacon_pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/beacons_guaguagua", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                beacon_pose_array_ = *msg;
            });

        // Parse points file
        parse_points_file("/home/user/Eurobot-2025-Navigation2-ws/install/navigation2_run/share/navigation2_run/params/script.yaml");

        // Open file path correctly
        std::string timestamp = get_timestamped_filename();
        std::string path = std::getenv("HOME") + std::string("/Eurobot-2025-Navigation2-ws/data_") + timestamp + ".csv";

        // Check if file is empty (only write header if it is)
        std::ifstream infile(path);
        bool is_empty = infile.peek() == std::ifstream::traits_type::eof();
        infile.close();

        file_.open(path, std::ios::app);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
        } else if (is_empty) {
            file_ << "index,timestamp,mode,goal_x,goal_y,goal_z,goal_w,final_x,final_y,lidar_x,lidar_y,beacon1_x,beacon1_y,beacon2_x,beacon2_y,beacon3_x,beacon3_y\n";
        }
    }


    ~ScriptSim() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void set_points_file(const std::string & points_file)
    {
        points_file_ = points_file;
    }

    bool execute_points()
    {
        for (const auto & point : points_)
        {
            const std::string & moving_type = point[0];
            double x = std::stod(point[1]); 
            double y = std::stod(point[2]);
            double z = std::stod(point[3]);
            double w = std::stod(point[4]);

            if (moving_type == "path" && !halt_)
            {
                std_msgs::msg::String controller_type;
                std_msgs::msg::String goal_checker_type;
                controller_type.data = "Fast";
                goal_checker_type.data = "Precise";
                controller_selector_pub_->publish(controller_type);
                goal_checker_selector_pub_->publish(goal_checker_type);

                send_navigation_goal(x, y, z, w);
            }
            else if (moving_type == "dock" && !halt_)
            {
                send_docking_goal(x, y, z, w);
            } 
            else if (moving_type == "wait" && !halt_)
            {
                wait(x);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown moving_type: %s", moving_type.c_str());
                return true;
            }

            while (halt_)
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for action to complete...");
                rclcpp::spin_some(this->get_node_base_interface());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }

        return true;
    }

private:
    void parse_points_file(const std::string & filename)
    {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["ScriptSim"] && config["ScriptSim"]["ros__parameters"] && config["ScriptSim"]["ros__parameters"]["points"])
        {
            points_ = config["ScriptSim"]["ros__parameters"]["points"].as<std::vector<std::vector<std::string>>>();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid YAML structure");
        }
    }

    void send_navigation_goal(double x, double y, double z, double w) {
        halt_ = true;

        if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.z = z;
        goal_msg.pose.pose.orientation.w = w;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Sending navigation goal to (%f, %f, %f, %f)", x, y, z, w);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this, x, y, z, w](const GoalHandleNavigate::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("ScriptSim"), "Navigation succeeded");
                index_++;
                if (file_.is_open() && beacon_pose_array_.poses.size() >= 3) {
                    auto stamp = this->now().seconds();
                    file_ << index_ << "," << stamp << ",Path,"
                          << x << "," << y << "," << z << "," << w << ","
                          << final_pose_data_.pose.pose.position.x << "," << final_pose_data_.pose.pose.position.y << ","
                          << lidar_pose_data_.pose.pose.position.x << "," << lidar_pose_data_.pose.pose.position.y << ","
                          << beacon_pose_array_.poses[0].position.x << "," << beacon_pose_array_.poses[0].position.y << ","
                          << beacon_pose_array_.poses[1].position.x << "," << beacon_pose_array_.poses[1].position.y << ","
                          << beacon_pose_array_.poses[2].position.x << "," << beacon_pose_array_.poses[2].position.y
                          << std::endl;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Skipping CSV write: not enough lidar pose data or file not open.");
                }

                halt_ = false;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("ScriptSim"), "Navigation failed");
                halt_ = false;
            }
        };

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }


    void send_docking_goal(double x, double y, double z, double w) {
        halt_ = true;

        if (!dock_robot_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "DockRobot action server not available");
            return;
        }

        auto goal_msg = DockRobot::Goal();
        goal_msg.use_dock_id = false;

        goal_msg.dock_pose.header.frame_id = "map";
        goal_msg.dock_pose.header.stamp = this->now();
        goal_msg.dock_pose.pose.position.x = x;
        goal_msg.dock_pose.pose.position.y = y;
        goal_msg.dock_pose.pose.orientation.z = z;
        goal_msg.dock_pose.pose.orientation.w = w;

        goal_msg.dock_pose.pose.position.z = 0.0;   // offset

        goal_msg.dock_type = "dock_slow_precise_x";

        goal_msg.navigate_to_staging_pose = false;

        RCLCPP_INFO(this->get_logger(), "Sending docking goal to (%f, %f, %f, %f)", x, y, z, w);

        auto send_goal_options = rclcpp_action::Client<DockRobot>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleDock::WrappedResult & result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("ScriptSim"), "Docking succeeded");
                halt_ = false;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("ScriptSim"), "Docking failed");
                halt_ = false;
            }
        };

        dock_robot_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void wait(float seconds)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for %f seconds", seconds);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
    }

    std::string points_file_;
    std::vector<std::vector<std::string>> points_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<DockRobot>::SharedPtr dock_robot_client_;
    bool halt_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_selector_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_checker_selector_pub_;

    geometry_msgs::msg::PoseWithCovarianceStamped final_pose_data_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr final_pose_sub_;
    geometry_msgs::msg::PoseArray beacon_pose_array_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr beacon_pose_array_sub_;
    geometry_msgs::msg::PoseWithCovarianceStamped lidar_pose_data_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_pose_sub_;

    std::ofstream file_;
    int index_ = 0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScriptSim>();
    std::string points_file(argv[1]);
    // RCLCPP_INFO(node->get_logger(), "points_file: %s", points_file.c_str());
    node->set_points_file(points_file);
    if(!node->execute_points())  rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}