#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include "model_lti_system.h"

class LTISystemNode : public rclcpp::Node
{
public:
    LTISystemNode()
      : Node("lti_model_node")
    {
        // 1) Declare and get parameter for the YAML path
        this->declare_parameter<std::string>("lti_model_yaml", "");
        std::string yaml_file = this->get_parameter("lti_model_yaml").as_string();

        // If user didn't supply a param, pick a default path
        if (yaml_file.empty()) {
            // Suppose your package is "control_lib_ros2"
            // and the file is "config/model_lti_system.yaml"
            yaml_file = ament_index_cpp::get_package_share_directory("control_lib_ros2") 
                        + "/config/config.yaml";
        }

        RCLCPP_INFO(this->get_logger(), "Loading LTI model from: %s", yaml_file.c_str());

        // 2) Load the model
        try {
            model_.loadFromFile(yaml_file);
            RCLCPP_INFO(this->get_logger(), "LTI model loaded successfully.");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load LTI model: %s", ex.what());
            // You might want to shutdown if critical
        }
        
        // 3) Setup subscriber/publisher
        using namespace std::placeholders;
        sub_input_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "optimal_control_input", 10, std::bind(&LTISystemNode::onInputReceivedDebug, this, _1));
        
        pub_output_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "measured_output", 10);

        pub_state_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("lti_state", 10); // NEW


        RCLCPP_INFO(this->get_logger(), "LTISystemNode started.");
    }

private:
    void onInputReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Convert msg->data to an Eigen vector
        Eigen::VectorXd input(msg->data.size());
        for (size_t i = 0; i < msg->data.size(); i++) {
            input(i) = msg->data[i];
        }

        // Step the model
        try {
            Eigen::VectorXd output = model_.step(input);
            
            // Publish output
            std_msgs::msg::Float64MultiArray out_msg;
            out_msg.data.resize(output.size());
            for (int i = 0; i < output.size(); i++) {
                out_msg.data[i] = output(i);
            }
            pub_output_->publish(out_msg);

            // Publish state
            Eigen::VectorXd state = model_.getState(); 
            std_msgs::msg::Float64MultiArray state_msg;
            state_msg.data.resize(state.size());
            for (int i = 0; i < state.size(); i++) {
                state_msg.data[i] = state(i);
            }
            pub_state_->publish(state_msg);

            //RCLCPP_INFO(this->get_logger(), "Stepped LTI system => output size %d", (int)output.size());
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Error stepping LTI system: %s", ex.what());
        }
    }

    void onInputReceivedDebug(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Convert msg->data to an Eigen vector
        Eigen::VectorXd input(msg->data.size());
        for (size_t i = 0; i < msg->data.size(); i++) {
            input(i) = msg->data[i];
        }

        try {
            // Ensure input size matches model expectations
            if (input.size() != model_.getInputSize()) {
                RCLCPP_ERROR(this->get_logger(), "Input size mismatch: expected %d, got %d",
                            model_.getInputSize(), (int)input.size());
                return;
            }

            // Step the model
            Eigen::VectorXd output = model_.step(input);
            //Eigen::VectorXd output = model_.stepWithNoise(input, 0.001);
            Eigen::VectorXd state = model_.getState(); 

            // Publish output
            std_msgs::msg::Float64MultiArray out_msg;
            out_msg.data.resize(output.size());
            for (int i = 0; i < output.size(); i++) {
                out_msg.data[i] = output(i);
            }
            pub_output_->publish(out_msg);

            // Log state and output
            std::ostringstream state_stream, output_stream;
            state_stream << "[";
            output_stream << "[";
            for (int i = 0; i < state.size(); i++) {
                state_stream << state(i) << (i < state.size() - 1 ? ", " : "");
            }
            for (int i = 0; i < output.size(); i++) {
                output_stream << output(i) << (i < output.size() - 1 ? ", " : "");
            }
            state_stream << "]";
            output_stream << "]";

            RCLCPP_INFO(this->get_logger(), "State: %s", state_stream.str().c_str());
            RCLCPP_INFO(this->get_logger(), "Output: %s", output_stream.str().c_str());

                        // Publish state
            std_msgs::msg::Float64MultiArray state_msg;
            state_msg.data.resize(state.size());
            for (int i = 0; i < state.size(); i++) {
                state_msg.data[i] = state(i);
            }
            pub_state_->publish(state_msg);


        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Error stepping LTI system: %s", ex.what());
        }
    }

    // LTI model instance
    LTIModelSystem model_;

    // ROS 2
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_input_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_output_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_; 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LTISystemNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
