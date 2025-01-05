#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>
#include <array>
#include <stdexcept>

// Include our circular_buffer_vector
#include "circular_buffer.h"

class ControllerNodeBase : public rclcpp::Node
{
public:
    ControllerNodeBase(const std::string &node_name, const std::string &config_file_name)
        : Node(node_name)
    {
        // 1) Load YAML configuration
        loadConfiguration(config_file_name);

        // 2) Now that we know nr_inputs_ and nr_outputs_, reinit the buffers
        //    We'll store inputs, derivatives, second_derivatives as dimension=nr_inputs_
        //    and targets as dimension=nr_outputs_, for example.
        //inputs_buffer_.reinit(nr_inputs_, 0.0);
        //derivatives_buffer_.reinit(nr_inputs_, 0.0);
        //second_derivatives_buffer_.reinit(nr_inputs_, 0.0);
        //targets_buffer_.reinit(nr_outputs_, 0.0);

        // 3) Setup publishers and subscribers
        setupInterfaces();

        // 4) Create a timer to run at fixed period delta_t_
        auto period_ns = std::chrono::duration<double>(delta_t_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period_ns),
            std::bind(&ControllerNodeBase::controlLoopCallback, this));

        RCLCPP_INFO(this->get_logger(),
                    "ControllerNodeBase timer created with delta_t=%.4f seconds", delta_t_);
    }

protected:
    // Number of inputs/outputs used by the control system
    int nr_inputs_ = 0;
    int nr_outputs_ = 0;

    // The main control function - must be overridden by derived classes
    virtual void computeControl(
        const std::vector<double> &inputs,
        const std::vector<double> &derivatives,
        const std::vector<double> &second_derivatives,
        const std::vector<double> &targets) = 0;

    // Buffers for storing the latest messages
    // NOTE: We do not know the dimension until after YAML load, so we do a default init here,
    // and then we call reinit() in the constructor body.
    //tools::CircularBufferVector<1024> inputs_buffer_;
    //tools::CircularBufferVector<1024> derivatives_buffer_;
    //tools::CircularBufferVector<1024> second_derivatives_buffer_;
    //tools::CircularBufferVector<1024> targets_buffer_;

    tools::CircularBuffer<double,1024> inputs_buffer_;
    tools::CircularBuffer<double,1024> derivatives_buffer_;
    tools::CircularBuffer<double,1024> second_derivatives_buffer_;
    tools::CircularBuffer<double,1024> targets_buffer_;


    // Publishers/Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_input_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_derivative_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_second_derivative_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_target_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_output_;

    // We store the YAML config if needed by derived classes
    YAML::Node config_;

    // The fixed control loop period (seconds)
    double delta_t_ = 0.01;

    // A timer that triggers controlLoopCallback() every delta_t_
    rclcpp::TimerBase::SharedPtr timer_;

private:
    // 1) Load configuration from YAML
    void loadConfiguration(const std::string &config_file_name)
    {
        std::string default_config_path =
            ament_index_cpp::get_package_share_directory("control_lib_ros2") +
            "/config/" + config_file_name;

        RCLCPP_INFO(this->get_logger(), "Loading YAML config: %s", default_config_path.c_str());

        try
        {
            config_ = YAML::LoadFile(default_config_path);

            // For example, "control_system: { nr_inputs: 3, nr_outputs: 1, delta_t: 0.01 }"
            nr_inputs_ = config_["control_system"]["nr_inputs"].as<int>();
            nr_outputs_ = config_["control_system"]["nr_outputs"].as<int>();

            // If missing, default to 0.01
            delta_t_ = config_["control_system"]["delta_t"].as<double>();

            RCLCPP_INFO(this->get_logger(),
                        "Loaded from YAML: nr_inputs=%d, nr_outputs=%d, delta_t=%.3f",
                        nr_inputs_, nr_outputs_, delta_t_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to load YAML config: %s. Using defaults...", e.what());
            throw;
        }
    }

    // 2) Setup publishers and subscribers
    void setupInterfaces()
    {
        using namespace std::placeholders;

        sub_input_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "measured_output", 10,
            std::bind(&ControllerNodeBase::onInputReceived, this, _1));

        sub_derivative_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "measured_output_derivative", 10,
            std::bind(&ControllerNodeBase::onDerivativeReceived, this, _1));

        sub_second_derivative_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "measured_output_second_derivative", 10,
            std::bind(&ControllerNodeBase::onSecondDerivativeReceived, this, _1));

        sub_target_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "control_target", 10,
            std::bind(&ControllerNodeBase::onTargetReceived, this, _1));

        pub_output_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("optimal_control_input", 10);

        RCLCPP_INFO(this->get_logger(),
                    "ControllerNodeBase initialized with %d inputs and %d outputs.",
                    nr_inputs_, nr_outputs_);
    }

    // 3) Timer Callback: run at fixed frequency (1/delta_t_)
    void controlLoopCallback()
    {
        // Example: retrieve the mean of the last 5 samples
        std::vector<double>  lastInputs = inputs_buffer_.get_snapshot(5);
        std::vector<double>  lastDerivs = derivatives_buffer_.get_snapshot(5);
        std::vector<double>  lastSecondDerivs = second_derivatives_buffer_.get_snapshot(5);
        std::vector<double>  lastTargets = targets_buffer_.get_snapshot(5);

        // Call the derived class's computeControl with these arrays
        computeControl(lastInputs, lastDerivs, lastSecondDerivs, lastTargets);
    }

protected:
    // 4) Default subscriber callbacks store the data in the circular buffers
    virtual void onInputReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
    {
        inputs_buffer_.add(msg->data[0]);
        std::cout << "mean input" << inputs_buffer_.compute_mean(10) << std::endl;
    }

    virtual void onDerivativeReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
    {
        derivatives_buffer_.add(msg->data[0]);
    }

    virtual void onSecondDerivativeReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
    {
        second_derivatives_buffer_.add(msg->data[0]);
    }

    virtual void onTargetReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
    {
        targets_buffer_.add(msg->data[0]);
    }
};
