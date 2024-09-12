#ifndef GO2_CONTROLLER__GO2_CONTROLLER_HPP_
#define GO2_CONTROLLER__GO2_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "go2_interfaces/msg/low_cmd.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

namespace go2_controller
{
    using lowCmd = go2_interfaces::msg::LowCmd;

    class Go2Controller : public controller_interface::ControllerInterface
    {
    public:
        // GO2_CONTROLLER_PUBLIC
        Go2Controller();

        // GO2_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        // GO2_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // GO2_CONTROLLER_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // GO2_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        // GO2_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        // // GO2_CONTROLLER_PUBLIC
        // controller_interface::CallbackReturn on_cleanup(
        //     const rclcpp_lifecycle::State &previous_state) override;

        // // GO2_CONTROLLER_PUBLIC
        // controller_interface::CallbackReturn on_error(
        //     const rclcpp_lifecycle::State &previous_state) override;

        // // GO2_CONTROLLER_PUBLIC
        // controller_interface::CallbackReturn on_shutdown(
        //     const rclcpp_lifecycle::State &previous_state) override;

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        double q[12];
        double dq[12];
        double effort[12];
        double kp[12];
        double kd[12];
        double tau[12];

        double commanded_effort[12];
        double q_e[12];
        double dq_e[12];

        double qr[12];
        double dqr[12];

        double Kp_gain;
        double Kd_gain;

        const std::vector<std::string> allowed_state_interface_types_ = {
            hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY,
            hardware_interface::HW_IF_EFFORT,
        };

        const std::vector<std::string> allowed_command_interface_types_ = {
            hardware_interface::HW_IF_EFFORT,
        };

        template <typename T>
        using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

        InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
        InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

        std::mutex mutex_controller;

        rclcpp::Subscription<lowCmd>::SharedPtr joints_reference_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_control_publisher_;

        float x = 0;
        float _startPos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

        // float _targetPos_1[12] = {0.0, 0, 0, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

        void publish_joint_control_signal()
        {
            auto message = std_msgs::msg::Float64MultiArray();
            for(long unsigned int i{0}; i < sizeof(commanded_effort)/sizeof(double); i++)
                message.data.push_back(commanded_effort[i]);

            joints_control_publisher_->publish(message);
        }
    };

}
#endif