#ifndef GO2_RGC__GO2_RGC_HPP_
#define GO2_RGC__GO2_RGC_HPP_

#include <vector>
#include <string>
#include <mutex>
#include <iostream>
#include <filesystem>

#include "controller_interface/controller_interface.hpp"

#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"



namespace go2_rgc
{
    using lowCmd = unitree_go::msg::LowCmd;
    using lowStates = unitree_go::msg::LowState;

    class Go2RGC : public controller_interface::ControllerInterface
    {
    public:
        // GO2_RGC_PUBLIC
        Go2RGC();

        // GO2_RGC_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        // GO2_RGC_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // GO2_RGC_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // GO2_RGC_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        // GO2_RGC_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_RGC_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_RGC_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        void computeLinearizedModel();

    protected:
        // Jacobianos usados no controle (ignorando base)
        Eigen::MatrixXd Jc;    // Jacobiano de contato (12x12)
        Eigen::MatrixXd Jcom;  // Jacobiano do centro de massa (3x12)

        // Função para calcular esses jacobianos
        void computeJacobians(const Eigen::VectorXd &q);
        pinocchio::Model model;
        std::shared_ptr<pinocchio::Data> data;

        Eigen::VectorXd _q;
        Eigen::VectorXd _qd;
        Eigen::VectorXd _tau;
        Eigen::VectorXd _effort;

        std::vector<double> kp;
        std::vector<double> kd;
        std::vector<double> ki;
        Eigen::VectorXd q_e;
        Eigen::VectorXd qi_e;
        Eigen::VectorXd dq_e;

        Eigen::VectorXd qr;
        Eigen::VectorXd dqr;

        int update_rate;

        lowCmd lowCmd_msg;

        double _percent;
        double _duration;
        bool _started;
        std::vector<double> _startPos;
        std::vector<double> _targetPos;
        uint32_t _lowTick;

        rclcpp::Publisher<lowCmd>::SharedPtr joints_cmd_publisher_;

        rclcpp::Subscription<lowCmd>::SharedPtr controller_reference_subscriber_;
        rclcpp::Subscription<lowStates>::SharedPtr lowstate_subscriber_;

        uint32_t control_mode;

        std::mutex mutex_controller;

        std::vector<int> _frame_index; 
        std::vector<std::string> _frames_names = {
            "1_FR_hip",  "1_FR_thigh",  "1_FR_calf",  "1_FR_foot",
            "2_FL_hip",  "2_FL_thigh",  "2_FL_calf",  "2_FL_foot",
            "3_RR_hip",  "3_RR_thigh",  "3_RR_calf",  "3_RR_foot",
            "4_RL_hip",  "4_RL_thigh",  "4_RL_calf",  "4_RL_foot"
        };
        // Matrizes do modelo linearizado
        Eigen::MatrixXd A_;
        Eigen::MatrixXd B_;

     
        Eigen::MatrixXd Jcom_linear;

        // Centro de massa e orientação
        Eigen::Vector3d com;
        Eigen::Quaterniond Q_;

        // Funções auxiliares
        Eigen::Matrix<double, 4, 3> rpy2Q(const Eigen::Quaterniond& Q);


    };

}
#endif