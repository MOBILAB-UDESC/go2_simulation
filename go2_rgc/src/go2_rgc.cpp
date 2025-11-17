#include "go2_rgc/go2_rgc.hpp"
#include <string>
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include <Eigen/SVD>   

#include "osqp++.h"

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
        // Inicialização explícita (redundante se já inicializadas no header)
        N = 15;
        M = 5;
        ts = 0.01;
        nx = 26;
        nu = 12;
        ny = 5;
        nc = 22;

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

        // inicializa total_mass_ a partir do modelo
        this->total_mass_ = this->getTotalMass();
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

    controller_interface::CallbackReturn Go2RGC::on_configure(const rclcpp_lifecycle::State &)
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

    controller_interface::return_type Go2RGC::update( const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        const auto logger = get_node()->get_logger();
        try
        {
            Eigen::Vector3d base(0, 0, 0);  // ou a posição real se tiver, começa posição 0,0,0
            Eigen::Quaterniond Q_base(1, 0, 0, 0);  // orientação do torso em quaternion (w, x, y, z)

            // Corrigir
            Eigen::VectorXd q(model.nq);
            q.head<3>() = base; // b -> base 
            q.segment<4>(3) << Q_base.x(), Q_base.y(), Q_base.z(), Q_base.w(); // w, x, y, z
            q.tail<12>() = _q; 
            pinocchio::forwardKinematics(model, *data, q);
            pinocchio::framesForwardKinematics(model, *data, q);
            pinocchio::updateGlobalPlacements(model, *data);

            // --- Centro de Massa (CoM) ---
            Eigen::Vector3d com = pinocchio::centerOfMass(model, *data, q);
    
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


            // Jcom (3x12)
            Eigen::MatrixXd Jcom_full = pinocchio::jacobianCenterOfMass(model, *data, q);
            // Remove os 6 DoF da base → pega apenas as colunas das juntas
            Jcom = Jcom_full.block(0, 3, 3, 12); // 3 linhas (x,y,z), 12 colunas (juntas)


            // gamma e Sa
            gamma.resize(12, 12);
            gamma.setZero();
            Sa.resize(3, 12);
            Sa.setZero();
            for (int i = 0; i < 4; ++i)
            {
                // Extrair posição do pé i
                int frame_id = _frame_index[i * 4 + 3];
                Eigen::Vector3d foot_pos = data->oMf[frame_id].translation(); // verificar 
                // Gamma linha i = Jcom - Jc linha i
                gamma.block(3 * i, 0, 3, 12) = Jcom - Jc.block(3 * i, 0, 3, 12);
                // Sa coluna i = skew(foot_pos - com)
                Eigen::Matrix3d mat = skewSymmetric(foot_pos - com);  // CORRETO
                Sa.block(0, 3 * i, 3, 3) = mat ;
            }
    
            // Jc_inv        
            Jc_inv = Jc.inverse();   

            // I_sum e Ib_inv 
            Eigen::MatrixXd I_sum = Eigen::MatrixXd::Zero(3, 12);  // [I I I I]
            I_sum.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
            I_sum.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
            I_sum.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity();
            I_sum.block(0, 9, 3, 3) = Eigen::Matrix3d::Identity();

            Eigen::Matrix3d Ib_base = pinocchio::crba(model, *data, q).block<3,3>(3,3);  // usa CRBA
            auto Ib_inv = Ib_base.inverse();

            // PARTE QUE PEGUEI DO COMPUTE LINEARIZED JACOBIAN 

            // Eigen::MatrixXd SF = I_sum * this->Jc_inv.traspose() / total_mass_;
            // Eigen::MatrixXd SM = Ib_inv * S * this->J_inv.transpose();

            
            // this->computeJacobians(q);
            // this->computeLinearizedModel(q);
            
            double Kp = 100.0;
            double Kd = 10.0;

            Eigen::MatrixXd k1;
            k1.resize(3,3);
            k1.setZero();

            k1 = (Kp / total_mass_)*I_sum*Jc_inv;
            auto k2 = (Kd / total_mass_)*I_sum*Jc_inv;
            auto k3 = Kp*Ib_inv*Sa*Jc_inv;
            auto k4 = Kd*Ib_inv*Sa*Jc_inv;

            gamma = gamma.inverse();
            Eigen::MatrixXd gamma_l_star, gamma_a_star;
            gamma_l_star.setZero(12, 3);
            gamma_a_star.setZero(12, 3);

            for(int i=0; i<4; i++)
            {
                gamma_l_star += gamma.block(0, 3*i, 12, 3);
                gamma_a_star += gamma.block(0, 3*i, 12,3)*Sa.block(0, 3*i, 3,3);
            }

            // 4. Matriz A (26x26)
            A_.resize(n_x, n_x);
            A_.setZero();

            // // Preenche blocos conforme a equação da imagem
            A_.block(0, 0, 3, 3) = k2 * gamma_l_star;  // -K₂Γ₁*
            A_.block(0, 3, 3, 3) = -k2 * gamma_a_star;    // K₂Γₐ*
            A_.block(0, 6, 3, n_j) = k1;                // -K₁

            A_.block(3, 0, 3, 3) = k4 * gamma_l_star;   // -K₄Γ₁*
            A_.block(3, 3, 3, 3) = -k4 * gamma_a_star;    // K₄Γₐ*
            A_.block(3, 6, 3, n_j) = k3;                // -K₃

            A_.block(6, 0, n_j, 3) = gamma_l_star;       // Γ₁*
            A_.block(6, 3, n_j, 3) = -gamma_a_star;     // -Γₐ*
            A_.block(18, 0, 3, 3) = Eigen::Matrix3d::Identity();  // I (integra r_dot -> r)
            A_.block(21, 3, 4, 3) = rpy2Q(Q_);              // T_ε (integra ω -> ε)

            // 5. Matriz B - Apenas B_u (K₁ e K₃)
            B_.resize(n_x, n_j);
            B_.setZero();
            B_.block(0, 0, 3, n_j) = -k1;  // K₁
            B_.block(3, 0, 3, n_j) = -k3;  // K₃

            // --- Discretização
            Aa.resize(n_x + n_j, n_x + n_j);  // 38 x 38
            Aa.setZero();

            Ba.resize(n_x + n_j, n_j);      // 38 x 12
            Ba.setZero();

            Ca.resize(n_y, n_x + n_j);  // 5 x 38
            Ca.setZero();

            Aa.block(0, 0, n_x, n_x) = Eigen::MatrixXd::Identity(n_x, n_x) + ts * A_;  // topo esquerdo
            Aa.block(0, n_x, n_x, n_j) = ts * B_;                                      // topo direito
            Aa.block(n_x, n_x, n_j, n_j) = Eigen::MatrixXd::Identity(n_j, n_j);        // canto inferior direito

            Ba.block(0, 0, n_x, n_j) = ts * B_;                                      // parte de cima
            Ba.block(n_x, 0, n_j, n_j) = Eigen::MatrixXd::Identity(n_j, n_j);        // parte de baixo

            Ca(0, 20) = 1;
            Ca.block(1, 21, 4, 4) = Eigen::MatrixXd::Identity(4, 4);

            // Matrizes de restrição
            G_cons = Eigen::MatrixXd::Zero(nc * N, n_u * M);
            G = Eigen::MatrixXd::Zero(ny * N, n_u * M);

            // Recebe valores das constraints
            std::tie(aux_cons, Phi_cons) = define_constraints_matrices();

            // Inicialização: primeira linha
            aux.resize(n_y, n_j); 
            aux = Ca * Ba;
            Phi.resize(n_y * N, n_x + n_u); 
            Phi.block(0, 0, n_y, n_x + n_u) = Ca * Aa;

            for (int i = 0; i < N; ++i)
            {
                int j = 0;
                if (i != 0)
                {
                    Phi.block(i * ny, 0, n_y, n_x + n_u) = Phi.block((i - 1) * n_y, 0, n_y, n_x + n_u) * Aa;
                    aux = Phi.block((i - 1) * ny, 0, ny, nx + nu) * Ba;

                    Phi_cons.block(i * nc, 0, nc, nx + nu) = Phi_cons.block((i - 1) * nc, 0, nc, nx + nu) * Aa;
                    aux_cons = Phi_cons.block((i - 1) * nc, 0, nc, nx + nu) * Ba;
                }

                while(j < M && (i + j) < N)
                {
                    G.block((i + j) * ny, j * nu, ny, nu) = aux;
                    G_cons.block((i + j) * nc, j * nu, nc, nu) = aux_cons;
                    j++;
                }
            }

            osqp::OsqpInstance instance;
            instance.objective_matrix = Eigen::SparseMatrix<double>(1, 1);
            instance.objective_vector.resize(1);
            instance.objective_vector << -1.0;
            instance.constraint_matrix = Eigen::SparseMatrix<double>(2, 2);
            instance.lower_bounds.resize(2);
            instance.lower_bounds << 0.0, 0.0;
            instance.upper_bounds.resize(2);
            instance.upper_bounds << 1.0, 1.0;

            osqp::OsqpSettings settings;
            osqp::OsqpSolver solver;

            solver.Init(instance, settings);

            // solver.Solve();
            

            // solver.setup(P, q, A, l, u);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "Exception in update(): %s", e.what());
            return controller_interface::return_type::ERROR;
        }
      
        return controller_interface::return_type::OK;
    }


    Eigen::Matrix3d Go2RGC::skewSymmetric(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d mat;
        mat <<     0, -v.z(),  v.y(),
                v.z(),     0, -v.x(),
            -v.y(),  v.x(),     0;
        return mat;
    }


    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> Go2RGC::define_constraints_matrices()
    {
        Eigen::MatrixXd aux_cons(n_c, n_j);         // exemplo
        Eigen::MatrixXd Phi_cons(n_c * N, n_x + n_j);

        aux_cons.setZero();
        Phi_cons.setZero();

        return std::make_tuple(aux_cons, Phi_cons);
    }


    Eigen::Matrix<double, 4, 3> Go2RGC::rpy2Q(const Eigen::Quaterniond& Q) 
    {
        Eigen::Matrix<double, 4, 3> T;
        T << -Q.w(),  Q.z(), -Q.y(),
            -Q.z(),  Q.w(),  Q.x(),
            Q.y(), -Q.x(), -Q.w(),
            -Q.x(), -Q.y(), -Q.z();
        return 0.5 * T;
    }


    double Go2RGC::getTotalMass() const
    {
        double total = 0.0;
        for (const auto &inertia : model.inertias)
        {
            // usa accessor mass() da Inertia do Pinocchio
            total += inertia.mass();
        }
        return total;
    }
}



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_rgc::Go2RGC,
    controller_interface::ControllerInterface)
