#include <chrono>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleSensor()
  : rclcpp_lifecycle::LifecycleNode("lifecycle_sensor")
  {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sensor_data", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&LifecycleSensor::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Sensor configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state)
  {
    publisher_->on_activate();
    rclcpp_lifecycle::LifecycleNode::on_activate(state);
    RCLCPP_INFO(this->get_logger(), "Sensor activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state)
  {
    publisher_->on_deactivate();
    rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(this->get_logger(), "Sensor deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    publisher_.reset();
    RCLCPP_INFO(this->get_logger(), "Sensor cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state)
  {
    timer_.reset();
    publisher_.reset();
    RCLCPP_INFO(this->get_logger(), "Sensor shutting down");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void timer_callback()
  {
    if (publisher_->is_activated()) {
      auto message = std_msgs::msg::Float64();
      
      // Generate random float between 0.0 and 100.0
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis(0.0, 100.0);
      message.data = dis(gen);
      
      RCLCPP_INFO(this->get_logger(), "Publishing sensor data: %.2f", message.data);
      publisher_->publish(message);
    }
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>> publisher_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<LifecycleSensor>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}