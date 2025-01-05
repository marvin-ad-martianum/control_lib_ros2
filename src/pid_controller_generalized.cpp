#include "controller_node_base.h"
#include "pid_controller.h"  // PIDControlSupervisor, etc.

class PIDControlNode : public ControllerNodeBase
{
public:
    PIDControlNode()
        : ControllerNodeBase("pid_controller_node", "config.yaml"),
          pid_supervisor_()
    {

        try {
            pid_supervisor_.loadConfigurationFromYamlNode(config_);
            RCLCPP_INFO(this->get_logger(), "Loaded PID config:" );
        } catch(const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse PID YAML: %s", ex.what());
        }

        pid_supervisor_.printStateMachine();

    }

protected:
    // Handle inputs
    /**void onInputReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg) override
    {

    }**/

    // Perform PID control computation
    void computeControlClean(
        const std::vector<double> &inputs,
        const std::vector<double> &derivatives,
        const std::vector<double> &second_derivatives,
        const std::vector<double> &targets) 
    {
        std_msgs::msg::Float64MultiArray output_msg;
        output_msg.data.resize(nr_outputs_);

        for (int i = 0; i < nr_outputs_; ++i)
        {
            double control = pid_supervisor_.computeControl(
                targets[i], inputs[i], derivatives[i], second_derivatives[i]);

            output_msg.data[i] = control;
        }

        pub_output_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "PID Control Output Published.");
    }

    void computeControl(
        const std::vector<double> &inputs,
        const std::vector<double> &derivatives,
        const std::vector<double> &second_derivatives,
        const std::vector<double> &targets) override
    {
        std_msgs::msg::Float64MultiArray output_msg;
        output_msg.data.resize(nr_outputs_);

        std::ostringstream debug_stream;
        debug_stream << "PID Control Debug:\n";

        for (int i = 0; i < nr_outputs_; ++i)
        {
            pid_supervisor_.updateGainsBasedOnTarget(targets[i]); // depending on dimension adapt.
            double control = pid_supervisor_.computeControl(
                targets[i], inputs[i], derivatives[i], second_derivatives[i]);

            output_msg.data[i] = control;
            auto error = pid_supervisor_.getErrorP();

            debug_stream << "Target[" << i << "]: " << targets[i]
                         << ", Input[" << i << "]: " << inputs[i]
                         << ", Derivative[" << i << "]: " << derivatives[i]
                         << ", Second Derivative[" << i << "]: " << second_derivatives[i]
                         << ", Error[" << i << "]: " << error
                         << ", Control[" << i << "]: " << control << "\n";
        }

        pub_output_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "PID Control Input Published.");
        RCLCPP_INFO(this->get_logger(), "%s", debug_stream.str().c_str());

    }

    

private:
    PIDControlSupervisor pid_supervisor_;
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControlNode>());
    rclcpp::shutdown();
    return 0;
}
