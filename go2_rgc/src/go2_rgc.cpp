#include "go2_rgc/go2_rgc.hpp"

#include <string>

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);


namespace go2_rgc
{

    Go2RGC::Go2RGC()
        : controller_interface::ControllerInterface()
        , model()
        , _q(12)
        , _qd(12)
        , _tau(12)
        , _effort(12)
        , kp(12)
        , kd(12)
        , ki(12)
        , q_e(12)
        , qi_e(12)
        , dq_e(12)
        , qr(12)
        , dqr(12)
        , update_rate(0)
        , _percent(0)
        , _duration(1000)
        , _started(false)
        , _startPos(12)
        , _targetPos(12)
        , _lowTick(0)
        , control_mode(1)
    {
        const auto package_share_path = ament_index_cpp::get_package_share_directory("go2_description");
        const auto xacro_path = std::filesystem::path(package_share_path) / "urdf" / "go2.xacro";
        const auto urdf_path = std::filesystem::temp_directory_path() / "go2.urdf";

        // Convert Xacro to URDF using ROS 2 xacro CLI
        std::string command = "ros2 run xacro xacro " + xacro_path.string() + " -o " + urdf_path.string();
        int result = std::system(command.c_str());

        if (result != 0)
        {
            std::cerr << "Error: Failed to convert Xacro to URDF!" << std::endl;
            return;
        }

        std::cout << "Converted Xacro to URDF: " << urdf_path << std::endl;
        
        // Create a set of Pinocchio models and data.
        pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model);

        model.gravity.linear(Eigen::Vector3d(0, 0, -9.8));
        data = std::make_shared<pinocchio::Data>(model);

        // instaciar o rgc_stup data
    }

    controller_interface::CallbackReturn Go2RGC::on_init()
    {
        try
        {
            auto_declare<std::vector<double>>("joints.initpos", _targetPos);
            auto_declare<std::string>("network_interface", "eth0");
            auto_declare<bool>("simulation", false);

            std::vector<double> zeros(12, 0.0);
            auto_declare<std::vector<double>>("gain.PD.Kp", zeros);
            auto_declare<std::vector<double>>("gain.PD.Kd", zeros);
            auto_declare<std::vector<double>>("gain.PDG.Kp", zeros);
            auto_declare<std::vector<double>>("gain.PDG.Kd", zeros);
            auto_declare<std::vector<double>>("gain.PID.Kp", zeros);
            auto_declare<std::vector<double>>("gain.PID.Kd", zeros);
            auto_declare<std::vector<double>>("gain.PID.Ki", zeros);
            auto_declare<std::vector<double>>("gain.PIDG.Kp", zeros);
            auto_declare<std::vector<double>>("gain.PIDG.Kd", zeros);
            auto_declare<std::vector<double>>("gain.PIDG.Ki", zeros);
            auto_declare<int>("control_mode", control_mode);
            
            auto_declare<int>("update_rate", update_rate);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    Go2RGC::command_interface_configuration() const
    {
        return {controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::InterfaceConfiguration
    Go2RGC::state_interface_configuration() const
    {
        return {controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::CallbackReturn
    Go2RGC::on_configure(
        const rclcpp_lifecycle::State &)
    {

        for (const auto& name : _frames_names) {
            if (model.existFrame(name)) {
                _frame_index.push_back(model.getFrameId(name));
            } else {
                std::cerr << "Warning: frame \"" << name << "\" not found!" << std::endl;
            }
        }

        // _frame_names
        // %_frame_idx

        auto logger = get_node()->get_logger();

        update_rate = get_node()->get_parameter("update_rate").get_value<int>();

        // TODO: use the name of topic from the YAML file
        lowstate_subscriber_ = get_node()->create_subscription<lowStates>(
            "/lowstate", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<lowStates> msg) -> void
            {
                std::lock_guard<std::mutex> lock(this->mutex_controller);

                for (auto index{0}; index < 12; index++)
                {
                    _q[index] = msg->motor_state[index].q;
                    _qd[index] = msg->motor_state[index].dq;
                }
                _lowTick = msg->tick;
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2RGC::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating Go2RGC...");

        // Wait for a valid reading from robot low states
        while(_lowTick == 0);

        for(int i=0; i<12; i++)
        {
            _startPos[i] = _q[i];
            qr[i] = _targetPos[i];
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2RGC::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating Go2RGC...");

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type Go2RGC::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        const auto logger = get_node()->get_logger();
        try
        {

            Eigen::Vector3d r_base(0, 0, 0);  // ou a posição real se tiver
            Eigen::Quaterniond Q_base(1, 0, 0, 0);  // orientação do torso em quaternion (w, x, y, z)

            // Corrigir q para ter 19 elementos
            Eigen::VectorXd q(model.nq);
            q.head<3>() = r_base;
            q.segment<4>(3) << Q_base.w(), Q_base.x(), Q_base.y(), Q_base.z();  // w, x, y, z
            q.tail<12>() = _q; 
            pinocchio::forwardKinematics(model, *data, q);
            pinocchio::framesForwardKinematics(model, *data, q);
            pinocchio::updateGlobalPlacements(model, *data);

            // Get homogeneous transform

            int aux = 0;
            for (const auto& name : _frames_names) {
            auto i = model.getFrameId(name);

            std::cout<<i<<"-"<< name <<std::endl;
            aux++;
            const pinocchio::SE3& H = data->oMf[i];
            // Print the transform
            std::cout << H.toHomogeneousMatrix() << std::endl;
            }
            // for (const auto& index : _frame_index) {


            // std::cout<<index <<std::endl;
            // const pinocchio::SE3& H = data->oMf[index];
            // // // Print the transform
            // std::cout << H.toHomogeneousMatrix() << std::endl;
            // }



          // 1. Estado base
        // Eigen::Vector3d r_base(0, 0, 0);
        // Eigen::Quaterniond Q_base(1, 0, 0, 0); // w, x, y, z

        // // 2. Montar q e v para Pinocchio
        // Eigen::VectorXd q(model.nq);
        // q.head<3>() = r_base;
        // q.segment<4>(3) = Eigen::Vector4d(Q_base.w(), Q_base.x(), Q_base.y(), Q_base.z());
        // q.tail<12>() = _q;

        // Eigen::VectorXd v(model.nv);
        // v.head<6>().setZero(); // Velocidade do torso
        // v.tail<12>() = _qd;

        // // 3. Atualizar Pinocchio
        // pinocchio::forwardKinematics(model, *data, q, v);
        // pinocchio::updateFramePlacements(model, *data);

        // // 4. Atualizar modelo RGC e matrizes A/B
        // rgc_model_->updateState(q, v, r_base, Q_base);
        
        // 5. Resolver o controle (em construção)
        // Eigen::VectorXd tau_command = rgc_model_->solveMPC();

        // 6. Publicar (ainda a definir)
        // ref_pub_->publish(...);
            return controller_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Exception in update(): %s", e.what());
            return controller_interface::return_type::ERROR;
        }
    }

    void RGCModel::computeLinearizedModel() {
    const int n_j = 7;  // Número de juntas ativas (ajuste conforme seu robô)
    const int n_x = 17; // Dimensão do estado: [r_dot (3), ω (3), q (7), r (3), ε (4)]
    
    // 1. Jacobiano de contato concatenado (Jc)
    Eigen::MatrixXd Jc(3 * contact_frames_.size(), model_.nv);
    for (size_t i = 0; i < contact_frames_.size(); ++i) {
        pinocchio::Data::Matrix6x J(6, model_.nv);
        pinocchio::computeFrameJacobian(model_, data_, q_, 
                                      model_.getFrameId(contact_frames_[i]), 
                                      pinocchio::LOCAL_WORLD_ALIGNED, J);
        Jc.block(3 * i, 0, 3, model_.nv) = J.topRows<3>();
    }

    // 2. Matrizes Γ₁* e Γₐ* (equação 14 do artigo)
    Eigen::MatrixXd Gamma = Jc;  // Simplificado (Γ = Jc no artigo)
    Eigen::MatrixXd Gamma_1_star = Gamma.block(0, 0, 3, n_j);  // Primeiras 3 linhas
    Eigen::MatrixXd Gamma_a_star = Gamma.block(3, 0, 3, n_j);  // Próximas 3 linhas

    // 3. Matrizes K₁, K₂, K₃, K₄ (ganhos do controlador PD)
    double Kp = 100.0, Kd = 10.0;
    Eigen::MatrixXd K1 = Eigen::MatrixXd::Identity(3, n_j) * Kp;
    Eigen::MatrixXd K2 = Eigen::MatrixXd::Identity(3, n_j) * Kd;
    Eigen::MatrixXd K3 = Eigen::MatrixXd::Identity(3, n_j) * Kp;
    Eigen::MatrixXd K4 = Eigen::MatrixXd::Identity(3, n_j) * Kd;

    // 4. Matriz A (17x17)
    A_.resize(n_x, n_x);
    A_.setZero();

    // Preenche blocos conforme a equação da imagem
    A_.block(0, 0, 3, 3) = -K2 * Gamma_1_star;  // -K₂Γ₁*
    A_.block(0, 3, 3, 3) = K2 * Gamma_a_star;    // K₂Γₐ*
    A_.block(0, 6, 3, n_j) = -K1;                // -K₁

    A_.block(3, 0, 3, 3) = -K4 * Gamma_1_star;   // -K₄Γ₁*
    A_.block(3, 3, 3, 3) = K4 * Gamma_a_star;    // K₄Γₐ*
    A_.block(3, 6, 3, n_j) = -K3;                // -K₃

    A_.block(6, 0, n_j, 3) = Gamma_1_star;       // Γ₁*
    A_.block(6, 3, n_j, 3) = -Gamma_a_star;      // -Γₐ*

    A_.block(9, 0, 3, 3) = Eigen::Matrix3d::Identity();  // I (integra r_dot -> r)
    A_.block(13, 3, 4, 3) = rpy2Q(Q_);              // T_ε (integra ω -> ε)

    // 5. Matriz B (17x7) - Apenas B_u (K₁ e K₃)
    B_.resize(n_x, n_j);
    B_.setZero();
    B_.block(0, 0, 3, n_j) = K1;  // K₁
    B_.block(3, 0, 3, n_j) = K3;  // K₃
}
}



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_rgc::Go2RGC,
    controller_interface::ControllerInterface)
