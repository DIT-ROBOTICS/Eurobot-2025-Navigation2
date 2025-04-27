#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <opennav_docking_msgs/action/dock_robot.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/string.hpp>

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

        // Parse points file
        parse_points_file("/home/user/Eurobot-2025-Navigation2-ws/install/navigation2_run/share/navigation2_run/params/script.yaml");
        // parse_points_file(points_file_);
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
            double w = std::stod(point[3]);

            if (moving_type == "path" && !halt_)
            {
                std_msgs::msg::String controller_type;
                std_msgs::msg::String goal_checker_type;
                controller_type.data = "Slow";
                goal_checker_type.data = "Precise";
                controller_selector_pub_->publish(controller_type);
                goal_checker_selector_pub_->publish(goal_checker_type);
                send_navigation_goal(x, y, w);
            }
            else if (moving_type == "dock" && !halt_)
            {
                send_docking_goal(x, y, w);
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

    void send_navigation_goal(double x, double y, double w)
    {
        halt_ = true;

        if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = w;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Sending navigation goal to (%f, %f, %f)", x, y, w);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNavigate::WrappedResult & result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Navigation succeeded");
                halt_ = false;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Navigation failed");
                halt_ = false;
            }
        };

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void send_docking_goal(double x, double y, double w)
    {
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
        goal_msg.dock_pose.pose.orientation.w = w;

        RCLCPP_INFO(this->get_logger(), "Sending docking goal to (%f, %f, %f)", x, y, w);

        auto send_goal_options = rclcpp_action::Client<DockRobot>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleDock::WrappedResult & result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Docking succeeded");
                halt_ = false;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Docking failed");
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