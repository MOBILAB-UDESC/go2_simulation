#include "go2_rgc/go2_rgc.hpp"
#include <string>
#include "pinocchio/algorithm/crba.hpp"



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
            // computeJacobians(q);

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

            Eigen::Vector3d base(0, 0, 0);  // ou a posição real se tiver, começa posição 0,0,0
            Eigen::Quaterniond Q_base(1, 0, 0, 0);  // orientação do torso em quaternion (w, x, y, z)

            // Corrigir q para ter 19 elementos
            Eigen::VectorXd q(model.nq);
            q.head<3>() = base; // b -> base 
            q.segment<4>(3) << Q_base.x(), Q_base.y(), Q_base.z(), Q_base.w(); // w, x, y, z
            q.tail<12>() = _q; 
            pinocchio::forwardKinematics(model, *data, q);
            pinocchio::framesForwardKinematics(model, *data, q);
            pinocchio::updateGlobalPlacements(model, *data);

            // --- Centro de Massa (CoM) ---
            Eigen::Vector3d com = pinocchio::centerOfMass(model, *data, q);
            
            // this->computeJacobians(q);
            this->computeLinearizedModel(q);
;
            return controller_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Exception in update(): %s", e.what());
            return controller_interface::return_type::ERROR;
        }
       



    }

    void Go2RGC::computeLinearizedModel(const Eigen::VectorXd &q) {
    const int n_j = 7;  // Número de juntas ativas (ajuste conforme seu robô)
    const int n_x = 26; // Dimensão do estado: [r_dot (3), ω (3), q (7), r (3), ε (4)]
    
    Eigen::Vector3d base(0, 0, 0);
    

    // // 1. Jacobiano de contato concatenado (Jc)

    Eigen::MatrixXd Jcom_full = pinocchio::jacobianCenterOfMass(model, *data, q);
    // Remove os 6 DoF da base → pega apenas as colunas das juntas
    Jcom = Jcom_full.block(0, 6, 3, 12); // 3 linhas (x,y,z), 12 colunas (juntas)

    // Jacobiano de contato (um bloco 3x12 por pé = 12x12)
    const int num_contacts = 4;
    Jc.resize(3 * num_contacts, 12); // 12x12 no total
    Jc.setZero();

    for (int i = 0; i < num_contacts; ++i)
    {
        // ID do frame do pé (último frame de cada perna)
        int frame_id = _frame_index[i * 4 + 3];

        // Jacobiano 6x18 completo
        pinocchio::Data::Matrix6x Jframe(6, model.nv);
        Jframe.setZero();

        pinocchio::computeFrameJacobian(model, *data, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, Jframe);

        // Pegamos só as 3 primeiras linhas (linear) e as 12 colunas das juntas
        Eigen::MatrixXd J_leg = Jframe.topRows<3>().block(0, 6, 3, 12);

        // Inserimos na linha correspondente do Jc
        Jc.block(3 * i, 0, 3, 12) = J_leg;
    }


    // === Cálculo de Gamma ===
    Eigen::MatrixXd Gamma(12, 12);
    Gamma.block(0, 0, 3, 12) = Jcom;
    Gamma.block(3, 0, 3, 12) = Jcom;
    Gamma.block(6, 0, 3, 12) = Jcom;
    Gamma.block(9, 0, 3, 12) = Jcom;
    Gamma -= Jc;  // Gamma = [J_CoM; J_CoM; J_CoM; J_CoM] - Jc

    // === Inversa de Gamma ===
    Eigen::MatrixXd Gamma_inv = Gamma.inverse();
    // === Blocos Gamma_1_star e Gamma_a_star ===
    Eigen::MatrixXd Gamma_1_star = Gamma.block(0, 0, 3, 12);   // linhas 0-2
    Eigen::MatrixXd Gamma_a_star = Gamma.block(3, 0, 3, 12);   // linhas 3-5


    // === Cálculo de GAMMA_lin e GAMMA_ang ===
    Eigen::MatrixXd GAMMA_lin = Eigen::MatrixXd::Zero(12, 3);
    Eigen::MatrixXd GAMMA_ang = Eigen::MatrixXd::Zero(12, 3);

    // === Criação da matriz S_gamma ===
    // Assumindo que você já tem os frames dos pés em LOCAL_WORLD_ALIGNED
    std::vector<Eigen::Vector3d> foot_positions;

    for (int i = 0; i < 4; ++i) {
        int frame_id = _frame_index[i * 4 + 3];  // índice do pé
        const auto& foot_placement = data->oMf[frame_id];
        foot_positions.push_back(foot_placement.translation());
    }

    Eigen::MatrixXd S_gamma(3, 12);


    for (int i = 0; i < 4; ++i) {
        Eigen::Matrix3d cross;
        Eigen::Vector3d rel = foot_positions[i] - com;
        cross <<      0, -rel.z(),  rel.y(),
                rel.z(),       0, -rel.x(),
                -rel.y(),  rel.x(),       0;
        S_gamma.block(0, 3 * i, 3, 3) = cross;
    }

    // === Loop de soma GAMMA_lin e GAMMA_ang ===
    for (int i = 0; i < 4; ++i) {
        GAMMA_lin += Gamma_inv.block(0, 3 * i, 12, 3);
        GAMMA_ang += Gamma_inv.block(0, 3 * i, 12, 3) * S_gamma.block(0, 3 * i, 3, 3);
    }

     // === Massa total do robô (ajuste para o seu caso real) ===
        double M_total = 80.51;

        // === Matriz S (Ia) ===
        // S = [cross(foot_i - 2*r + b)]
        Eigen::MatrixXd S(3, 12);
        for (int i = 0; i < 4; ++i) {
            Eigen::Matrix3d cross;
            Eigen::Vector3d rel = foot_positions[i] - 2.0 * base + base; // (foot - 2*r + b)
            cross <<      0, -rel.z(),  rel.y(),
                    rel.z(),       0, -rel.x(),
                    -rel.y(),  rel.x(),       0;
            S.block(0, 3 * i, 3, 3) = cross;
        }

        // === Inércia rotacional da base (3x3) ===
        Eigen::Matrix3d Ib_base = pinocchio::crba(model, *data, q).block<3,3>(3,3);  // usa CRBA
        Eigen::Matrix3d Ib_inv = Ib_base.inverse();

        // === Inverso de Jc ===
        Eigen::MatrixXd inv_Jc = Jc.inverse().transpose(); // (Jc⁻¹)ᵗ = (Jcᵗ)⁻¹

        // === SF e SM ===
        Eigen::MatrixXd I_sum = Eigen::MatrixXd::Zero(3, 12);  // [I I I I]
        I_sum.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        I_sum.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
        I_sum.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity();
        I_sum.block(0, 9, 3, 3) = Eigen::Matrix3d::Identity();

        Eigen::MatrixXd SF = I_sum * inv_Jc / M_total;
        Eigen::MatrixXd SM = Ib_inv * S * inv_Jc;



        // === Inércia rotacional da base (3x3) ===
        pinocchio::crba(model, *data, q);  // Atualiza data->Ig corretamente
        Eigen::Matrix3d I = data->Ig.inertia();//.matrix().block<3,3>(0,0);  // Obtém a matriz de inércia 3x3
        Eigen::Matrix3d I_inv = I.inverse();  // Inversa
        // inversão direta (simples)
        if (I.fullPivLu().isInvertible()) {
            I_inv = I.inverse();
        } else {
            // fallback: regularização para evitar singularidade
            const double eps = 1e-8;
            I_inv = (I + eps * Eigen::Matrix3d::Identity()).inverse();
        }
        
      // === Inércia rotacional da base (3x3) ===


        double Kp = 100.0, Kd = 10.0;
        Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
        K1_.resize(3, 12);
        K2_.resize(3, 12);
        K3_.resize(3, 12);
        K4_.resize(3, 12);

        K1_ << I3, I3, I3, I3;
        K2_ << I3, I3, I3, I3;
        K3_ << I3, I3, I3, I3;
        K4_ << I3, I3, I3, I3;

        K1_ *= Kp;
        K2_ *= Kd;
        K3_ *= Kp;
        K4_ *= Kd;


            // --- Discretização simples (Euler)
        A_discrete_ = A_ * ts_ + Eigen::MatrixXd::Identity(A_.rows(), A_.cols());
        B_discrete_ = B_ * ts_;


    // 4. Matriz A (26x26)
    A_.resize(n_x, n_x);
    A_.setZero();


    std::cout<<"Gamma"<<std::endl; 
    std::cout << Gamma_1_star.rows()<<","<<Gamma_1_star.cols()  << std::endl;

    // Preenche blocos conforme a equação da imagem
    A_.block(0, 0, 3, 3) = -K2_ * Gamma_1_star;  // -K₂Γ₁*
    A_.block(0, 3, 3, 3) = K2_ * Gamma_a_star;    // K₂Γₐ*
    A_.block(0, 6, 3, n_j) = -K1_;                // -K₁

    A_.block(3, 0, 3, 3) = -K4_ * Gamma_1_star;   // -K₄Γ₁*
    A_.block(3, 3, 3, 3) = K4_ * Gamma_a_star;    // K₄Γₐ*
    A_.block(3, 6, 3, n_j) = -K3_;                // -K₃

    A_.block(6, 0, n_j, 3) = Gamma_1_star;       // Γ₁*
    A_.block(6, 3, n_j, 3) = -Gamma_a_star;      // -Γₐ*

    A_.block(9, 0, 3, 3) = Eigen::Matrix3d::Identity();  // I (integra r_dot -> r)
    A_.block(13, 3, 4, 3) = rpy2Q(Q_);              // T_ε (integra ω -> ε)

    // 5. Matriz B (17x7) - Apenas B_u (K₁ e K₃)
    B_.resize(n_x, n_j);
    B_.setZero();
    B_.block(0, 0, 3, n_j) = K1_;  // K₁
    B_.block(3, 0, 3, n_j) = K3_;  // K₃

        // === Impressão ===
   // std::cout << "Gamma:\n" << Gamma << std::endl;
   // std::cout << "Gamma_inv:\n" << Gamma_inv << std::endl;
   // std::cout << "GAMMA_lin:\n" << GAMMA_lin << std::endl;
   // std::cout << "GAMMA_ang:\n" << GAMMA_ang << std::endl;
   // std::cout << "k1_:\n" << K1_ << std::endl;
    std::cout << "I:\n" << I << std::endl;
    std::cout << "Jc:\n" << Jc << std::endl;


            // === Impressão para debug ===
    //std::cout << "S:\n" << S << std::endl;
    //std::cout << "Ib_inv:\n" << Ib_inv << std::endl;
    //std::cout << "SF:\n" << SF << std::endl;
    //std::cout << "SM:\n" << SM << std::endl;
    //std::cout << "G_q:\n" << G_q_ << std::endl;
    //std::cout << "Phi_q:\n" << Phi_q_ << std::endl;

    // std::cout << "Converted Xacro to URDF: " << urdf_path << std::endl;

    // std::cout << "Center of Mass: " << com.transpose() << std::endl;

}

Eigen::Matrix<double, 4, 3> Go2RGC::rpy2Q(const Eigen::Quaterniond& Q) {
    Eigen::Matrix<double, 4, 3> T;
    T << -Q.x(), -Q.y(), -Q.z(),
          Q.w(), -Q.z(),  Q.y(),
          Q.z(),  Q.w(), -Q.x(),
         -Q.y(),  Q.x(),  Q.w();
    return 0.5 * T;
        // --- Montagem de A_ext e B_u_ext (Aumentado)
        int nx = A_discrete_.rows(); // 17
        int nu = B_discrete_.cols(); // 12

        A_ext_.resize(nx + nu, nx + nu);
        A_ext_.setZero();
        A_ext_.block(0, 0, nx, nx) = A_discrete_;
        A_ext_.block(0, nx, nx, nu) = B_discrete_;
        A_ext_.block(nx, nx, nu, nu) = Eigen::MatrixXd::Identity(nu, nu);

        B_u_ext_.resize(nx + nu, nu);
        B_u_ext_.setZero();
        B_u_ext_.block(0, 0, nx, nu) = B_discrete_;
        B_u_ext_.block(nx, 0, nu, nu) = Eigen::MatrixXd::Identity(nu, nu);

        // B_g não existe 


}
void Go2RGC::computeJacobians(const Eigen::VectorXd &q)
{


    // // Jacobiano de contato (um bloco 3x12 por pé)
    const int num_contacts = 4;


    for (int i = 0; i < num_contacts; ++i)
    {
        // ID do frame do pé (último frame de cada perna)
        int frame_id = _frame_index[i * 4 + 3];

        // Jacobiano 6x18 completo
        pinocchio::Data::Matrix6x Jframe(6, model.nv);
        Jframe.setZero();

        pinocchio::computeFrameJacobian(model, *data, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, Jframe);

        // Pegamos só as 3 primeiras linhas (linear) e as 12 colunas das juntas
        Eigen::MatrixXd J_leg = Jframe.topRows<3>().block(0, 6, 3, 12);

        // Inserimos na linha correspondente do Jc
        Jc.block(3 * i, 0, 3, 12) = J_leg;

    }


}

}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_rgc::Go2RGC,
    controller_interface::ControllerInterface)
