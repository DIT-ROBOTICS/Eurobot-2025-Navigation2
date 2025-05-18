#include "rclcpp/rclcpp.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"

class ReadySignalServer : public rclcpp::Node
{
public:
  ReadySignalServer() : Node("ready_signal_server")
  {
    server_ = this->create_service<btcpp_ros2_interfaces::srv::StartUpSrv>(
      "/robot/startup/ready_signal",
      std::bind(&ReadySignalServer::handleRequest, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ReadySignal service server started and ready!");
  }

private:
  rclcpp::Service<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr server_;

  void handleRequest(
    const std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Request> request,
    std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Response> response)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received ReadySignal Request -> group: %d, state: %d", request->group, request->state);

    response->group = request->group;
    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Sent response: success = true");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReadySignalServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
