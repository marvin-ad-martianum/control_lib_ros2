#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "pid_controller.h"  // Contains PIDControlSupervisor, etc.




class PIDcontrol : public rclcpp::Node
{
public:
  PIDcontrol()
  : Node("pid_controller_node"),
    derivative_(0.0),
    second_derivative_(0.0)
  {
    // 1) Compute default path (installed by control_lib into its share directory)
    std::string default_config_path =
      ament_index_cpp::get_package_share_directory("control_lib_ros2") +
      "/config/pid_config.yaml";

    RCLCPP_INFO(this->get_logger(), 
                "Default config path is: %s", default_config_path.c_str());

    // 2) Declare a parameter "pid_yaml" storing a PATH
    this->declare_parameter<std::string>("pid_yaml", default_config_path);

    // 3) Get the parameter (user may override at runtime)
    std::string pid_yaml_path = this->get_parameter("pid_yaml").as_string();

    RCLCPP_INFO(this->get_logger(), "Using YAML path: %s", pid_yaml_path.c_str());

    // 4) Load from YAML path
    try {
      pid_supervisor_.loadConfigurationFromYamlPath(pid_yaml_path);
      RCLCPP_INFO(this->get_logger(), 
                  "PID config loaded successfully from: %s", pid_yaml_path.c_str());
    } catch (const std::exception &ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to parse PID YAML: %s", ex.what());
    }


    // 5) Setup subscribers/publishers
    using namespace std::placeholders;
    sub_input_ = this->create_subscription<std_msgs::msg::Float32>(
        "control_input", 10, std::bind(&PIDcontrol::onControlInput, this, _1));

    sub_derivative_ = this->create_subscription<std_msgs::msg::Float32>(
        "control_derivative", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          derivative_ = msg->data;
        });

    sub_second_derivative_ = this->create_subscription<std_msgs::msg::Float32>(
        "control_second_derivative", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          second_derivative_ = msg->data;
        });

    pub_output_ = this->create_publisher<std_msgs::msg::Float32>("control_output", 10);

    pid_supervisor_.printStateMachine();
    RCLCPP_INFO(this->get_logger(), "PIDcontrol started.");
  }

private:
  void onControlInput(const std_msgs::msg::Float32::SharedPtr msg)
  {
    double measurement = static_cast<double>(msg->data);

    // Possibly switch gains if needed based on "reference"
    pid_supervisor_.updateGainsBasedOnTarget(target);

    // Compute control
    double control = pid_supervisor_.computeControl(
        reference,
        measurement,
        derivative_,
        second_derivative_);

    // Publish
    std_msgs::msg::Float32 out_msg;
    out_msg.data = static_cast<float>(control);
    pub_output_->publish(out_msg);

    RCLCPP_INFO(this->get_logger(),
                "input=%.2f, deriv=%.2f, 2nd_deriv=%.2f => control=%.2f",
                measurement, derivative_, second_derivative_, control);
  }

  // Our custom PID wrapper
  PIDControlSupervisor pid_supervisor_;

  double derivative_;
  double second_derivative_;

  // ROS 2 interfaces
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_input_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_derivative_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_second_derivative_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_output_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDcontrol>());
  rclcpp::shutdown();
  return 0;
}
