#include "go2_rgc/go2_rgc.hpp"
#include <string>
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include <Eigen/SVD>

#include "osqp++.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "rclcpp/rclcpp.hpp"
#include <unitree_go/msg/low_state.hpp>
#include "unitree_go/msg/low_cmd.hpp"

#include <pinocchio/algorithm/centroidal.hpp>

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

namespace go2_rgc
{

    Go2RGC::Go2RGC()
        : controller_interface::ControllerInterface(), model(), _q(12), _qd(12), _tau(12), _effort(12), kp(12), kd(12), ki(12), q_e(12), qi_e(12), dq_e(12), qr(12), dqr(12), update_rate(0), _percent(0), _duration(1000), _started(false), _startPos(12), _targetPos(12), _lowTick(0), control_mode(1)
    {
        // Inicialização explícita
        N = 15;
        M = 5;
        ts = 0.01;
        nx = 26;
        nu = 12;
        ny = 5;
        nc = 22;

        A_.resize(n_x, n_x);
        A_.setZero();
        A_(2, 25) = 1;
        A_.block(18, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

        B_.resize(n_x, n_j);
        B_.setZero();

        Aa.resize(n_x + n_j, n_x + n_j); // 38 x 38
        Aa.setZero();

        Ba.resize(n_x + n_j, n_j); // 38 x 12
        Ba.setZero();

        Ca.resize(n_y, n_x + n_j); // 5 x 38
        Ca.setZero();

        Ca(0, 20) = 1;
        Ca.block(1, 21, 4, 4) = Eigen::MatrixXd::Identity(4, 4);

        Q = Eigen::MatrixXd::Identity(ny * N, ny * N);
        R = Eigen::MatrixXd::Identity(nu * M, nu * M);
        l = Eigen::VectorXd::Constant(nc * N, -1.0); // exemplo
        u = Eigen::VectorXd::Constant(nc * N, 1.0);

        base_pos.resize(3);
        base_ori.resize(4);
        base_lin_vel.resize(3);
        base_ang_vel.resize(3);

        base_pos.setZero();
        base_ori.setZero();
        base_lin_vel.setZero();
        base_ang_vel.setZero();

        I_stack.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        I_stack.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
        I_stack.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity();
        I_stack.block(0, 9, 3, 3) = Eigen::Matrix3d::Identity();

        //  Creatimg the reference for the rz position and the body orientation
        ref.resize(n_y * N, 1);
        ref.setZero();

        double rz_ref = 0.25;
        Eigen::VectorXd Q_ref(4);
        Q_ref << 0, 0, 0, 1;

        Eigen::VectorXd _ref(5);
        _ref << rz_ref, Q_ref;
        for (int i = 0; i < N; ++i)
        {
            ref.segment<5>(i * 5) = _ref;
        }

        // Q weight matrices
        Eigen::MatrixXd _Q;
        _Q.resize(ny, ny);
        _Q.setZero();
        _Q(0, 0) = 0.8;
        _Q(1, 1) = 0.025;
        _Q(2, 2) = 0.025;
        _Q(3, 3) = 0.025;
        _Q(4, 4) = 0.025;
        for (int i = 0; i < N; ++i)
        {
            Q.block(i * 5, i * 5, 5, 5) = _Q; // q_zr, q_ep
        }

        // R weight matrices
        R = Eigen::MatrixXd::Identity(M * nu, M * nu);

        // Load URDF file

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

        for (const auto &name : _frames_names)
        {
            if (model.existFrame(name))
            {
                _frame_index.push_back(model.getFrameId(name));
            }
            else
            {
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

        odometry_subscriber_ = get_node()->create_subscription<odometry>(
            "/odom_gz", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<odometry> msg) -> void
            {
                // std::lock_guard<std::mutex> lock(this->mutex_controller);
                base_pos[0] = msg->pose.pose.position.x;
                base_pos[1] = msg->pose.pose.position.y;
                base_pos[2] = msg->pose.pose.position.z;

                base_ori[0] = msg->pose.pose.orientation.x;
                base_ori[1] = msg->pose.pose.orientation.y;
                base_ori[2] = msg->pose.pose.orientation.z;
                base_ori[3] = msg->pose.pose.orientation.w;

                base_lin_vel[0] = msg->twist.twist.linear.x;
                base_lin_vel[1] = msg->twist.twist.linear.y;
                base_lin_vel[2] = msg->twist.twist.linear.z;

                base_ang_vel[0] = msg->twist.twist.angular.x;
                base_ang_vel[1] = msg->twist.twist.angular.y;
                base_ang_vel[2] = msg->twist.twist.angular.z;
            });

        active_rgc_subscriber_ = get_node()->create_subscription<boolmsgs>(
            "/active_rgc", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<boolmsgs> msg) -> void
            {
                active_rgc = msg->data;
            });

        joints_cmd_publisher_ = get_node()->create_publisher<lowCmd>("/go2_jointcontroller/JointControllerReferences", 10);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2RGC::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating Go2RGC...");

        // Wait for a valid reading from robot low states
        while (_lowTick == 0)
            ;

        for (int i = 0; i < 12; i++)
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

    controller_interface::return_type Go2RGC::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        const auto logger = get_node()->get_logger();
        try
        {
            if (active_rgc)
            { // Corrigir
                if (first_iteration)
                {
                    for (int i; i < 12; i++)
                    {
                        qr[i] = _q[i];
                        first_iteration = false;
                    }
                }
                Eigen::VectorXd q(model.nq);
                q.head<3>() = base_pos;
                q.segment<4>(3) << base_ori;
                q.tail<12>() = _q;

                Eigen::VectorXd dq(model.nv);
                dq.setZero();
                dq.head<3>() = base_lin_vel; //
                dq.segment<3>(3) = base_ang_vel;
                dq.tail<12>() = _qd;

                // Forward kinematics
                pinocchio::forwardKinematics(model, *data, q);

                // Update frame placements
                pinocchio::updateFramePlacements(model, *data);

                // CCRBA (Composite Rigid Body Algorithm for centroidal)
                pinocchio::ccrba(model, *data, q, Eigen::VectorXd::Zero(model.nv));

                // Compute center of mass position
                Eigen::Vector3d r = pinocchio::centerOfMass(model, *data, q);

                // Evaluate the CoM velocity
                pinocchio::Force centroidal_momentum = pinocchio::computeCentroidalMomentum(model, *data, q, dq);
                Eigen::Vector3d dr = centroidal_momentum.linear() / data->mass[0];

                // Access the spatial inertia matrix
                pinocchio::Inertia I = data->Ig;

                auto Iinv = I.inertia().inverse();

                pinocchio::Data::Matrix3x J_com_full_body = pinocchio::jacobianCenterOfMass(model, *data, q, pinocchio::LOCAL_WORLD_ALIGNED);

                Eigen::MatrixXd J_com_full = J_com_full_body.rightCols(J_com_full_body.cols() - 6);

                Eigen::Matrix<double, 12, 12> J_com_stacked;

                J_com_stacked.block<3, 12>(0, 0) = J_com_full;
                J_com_stacked.block<3, 12>(3, 0) = J_com_full;
                J_com_stacked.block<3, 12>(6, 0) = J_com_full;
                J_com_stacked.block<3, 12>(9, 0) = J_com_full;

                // Jacobiano de contato (um bloco 3x12 por pé = 12x12)
                Eigen::Matrix<double, 4, 3> contacts;

                Jc.resize(12, 12); // 12x12 no total
                gamma.resize(12, 12);
                Sa.resize(12, 3);

                Jc.setZero();
                gamma.setZero();
                Sa.setZero();
                contacts.setZero();
                pinocchio::Data::Matrix6x Jframe(6, model.nv);
                for (int i = 0; i < 4; i++)
                {
                    int frame_id = _frame_index[i * 4 + 3];

                    Jframe.setZero();
                    pinocchio::computeFrameJacobian(model, *data, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, Jframe);
                    Eigen::MatrixXd J_leg = Jframe.topRows<3>().block(0, 6, 3, 12);
                    Jc.block<3, 12>(i * 3, 0) = J_leg;
                    Eigen::Vector3d contact_pos = data->oMf[frame_id].translation();
                    Sa.block<3, 3>(i * 3, 0) = skewSymmetric(contact_pos - r);
                    contacts.row(i) = contact_pos.transpose();
                }
                gamma = J_com_stacked - Jc;
                auto gamma_inv = gamma.inverse();

                auto gamma_l_star = gamma_inv * I_stack.transpose();
                auto gamma_a_star = gamma_inv * Sa;

                // Jc_inv
                Jc_inv = (Jc.transpose()).inverse();

                double Kp = 50.0;
                double Kd = 2.5;

                auto k1 = (Kp / total_mass_) * I_stack * Jc_inv;
                auto k2 = (Kd / total_mass_) * I_stack * Jc_inv;
                auto k3 = Kp * Iinv * -Sa.transpose() * Jc_inv;
                auto k4 = Kd * Iinv * -Sa.transpose() * Jc_inv;

                // Preenche blocos conforme a equação da imagem
                A_.block(0, 0, 3, 3) = k2 * gamma_l_star;
                A_.block(0, 3, 3, 3) = -k2 * gamma_a_star;
                A_.block(0, 6, 3, n_j) = k1;

                A_.block(3, 0, 3, 3) = k4 * gamma_l_star;
                A_.block(3, 3, 3, 3) = -k4 * gamma_a_star;
                A_.block(3, 6, 3, n_j) = k3;

                A_.block(6, 0, n_j, 3) = gamma_l_star;
                A_.block(6, 3, n_j, 3) = -gamma_a_star;

                A_.block(21, 3, 4, 3) = rpy2Q(base_ori);

                // 5. Matriz B - Apenas B_u (K₁ e K₃)
                B_.block(0, 0, 3, n_j) = -k1; // K₁
                B_.block(3, 0, 3, n_j) = -k3; // K₃

                // --- Discretização

                Aa.block(0, 0, n_x, n_x) = Eigen::MatrixXd::Identity(n_x, n_x) + ts * A_; // topo esquerdo
                Aa.block(0, n_x, n_x, n_j) = ts * B_;                                     // topo direito
                Aa.block(n_x, n_x, n_j, n_j) = Eigen::MatrixXd::Identity(n_j, n_j);       // canto inferior direito

                Ba.block(0, 0, n_x, n_j) = ts * B_;                               // parte de cima
                Ba.block(n_x, 0, n_j, n_j) = Eigen::MatrixXd::Identity(n_j, n_j); // parte de baixo

                Eigen::VectorXd x(38);

                x.segment(0, 3) = dr;
                x.segment(3, 3) = base_ang_vel;
                x.segment(6, 12) = _q;
                x.segment(18, 3) = r;
                x.segment(21, 4) = base_ori;
                x(25) = -9.81;
                x.segment(26, 12) = qr;

                // // Matrizes de restrição
                // G_cons = Eigen::MatrixXd::Zero(nc * N, n_u * M);
                G = Eigen::MatrixXd::Zero(ny * N, n_u * M);

                // // até linha 406 ulrimas modificações de 21/11
                // if (first_iteration)
                // {
                //     Eigen::VectorXd l0 = Eigen::VectorXd::Constant(nc, -0.2);
                //     Eigen::VectorXd u0 = Eigen::VectorXd::Constant(nc, 0.2);

                //     Eigen::VectorXd f_l = Eigen::VectorXd::Constant(nc - 2, 0.0);
                //     Eigen::VectorXd f_u = Eigen::VectorXd::Constant(nc - 2, 200.0);

                //     l = Eigen::VectorXd::Zero(nc);
                //     u = Eigen::VectorXd::Zero(nc);
                //     l << l0.head(2), f_l;
                //     u << u0.head(2), f_u;

                //     l = l.replicate(N, 1);
                //     u = u.replicate(N, 1);
                //     first_iteration = false;
                // }50

                // Vetores normais e tangentes dos pés (defina corretamente!)
                // Eigen::Vector3d n_fl, n_fr, n_rl, n_rr;
                // Eigen::Vector3d t1_fl, t1_fr, t1_rl, t1_rr;
                // Eigen::Vector3d t2_fl, t2_fr, t2_rl, t2_rr;
                // double mu = 0.7;

                // // TODO: Inicialize n_fl, t1_fl, etc. com base nos frames dos pés (LOCAL_WORLD_ALIGNED ou fixos)

                // // Cf individual

                // Cf_fl = cf_matrix(n_fl, t1_fl, t2_fl, mu);
                // Cf_fr = cf_matrix(n_fr, t1_fr, t2_fr, mu);
                // Cf_rl = cf_matrix(n_rl, t1_rl, t2_rl, mu);
                // Cf_rr = cf_matrix(n_rr, t1_rr, t2_rr, mu);

                // // Cf total (20x12)
                // Cf = Eigen::MatrixXd::Zero(20, 12);
                // Cf.block(0, 0, 5, 3) = Cf_fl;
                // Cf.block(5, 3, 5, 3) = Cf_fr;
                // Cf.block(10, 6, 5, 3) = Cf_rl;
                // Cf.block(15, 9, 5, 3) = Cf_rr;

                // // Fc_mtx = -Cf * Jc^-1
                // Fc_mtx = -Cf * Jc_inv;

                // // Atualização da constraint matrix
                // aux_cons.block(0, 0, 2, Ba.cols()) = C_cons.block(0, 0, 2, C_cons.cols()) * Ba;
                // aux_cons.block(2, 0, 20, Ba.cols()) = kp * Fc_mtx; // kp constante ou vetor → ajuste conforme

                // // C_cons parte inferior
                // C_cons.block(2, 0, 20, L.cols()) = Fc_mtx * L;

                // // Phi_cons
                // Phi_cons.block(0, 0, nc, Aa.cols()) = C_cons * Aa;

                // // Recebe valores das constraints
                // std::tie(aux_cons, Phi_cons) = define_constraints_matrices();

                // // Inicialização: primeira linha
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

                        // Phi_cons.block(i * nc, 0, nc, nx + nu) = Phi_cons.block((i - 1) * nc, 0, nc, nx + nu) * Aa;
                        // aux_cons = Phi_cons.block((i - 1) * nc, 0, nc, nx + nu) * Ba;
                    }

                    while (j < M && (i + j) < N)
                    {
                        G.block((i + j) * ny, j * nu, ny, nu) = aux;
                        // G_cons.block((i + j) * nc, j * nu, nc, nu) = aux_cons;
                        j++;
                    }
                }

                // // OSQP Solver
                // // Build cost

                Eigen::MatrixXd H_dense = G.transpose() * Q * G + R;

                // que = 2 * G^T * Q * (Phi * x - ref);
                Eigen::VectorXd diff = (Phi * x - ref);
                Eigen::VectorXd que = 2.0 * (G.transpose() * (Q * diff));

                Eigen::MatrixXd H_final = 2.0 * H_dense;

                // // Convert to sparse (CSC)
                Eigen::SparseMatrix<double> P = H_final.sparseView();

                // Eigen::SparseMatrix<double> A_cons = G_cons.sparseView();
                // // Adjust bounds
                // Eigen::VectorXd l_adj = l - (Phi_cons * x);
                // Eigen::VectorXd u_adj = u - (Phi_cons * x);

                Eigen::SparseMatrix<double> A_cons;
                A_cons.resize(0, P.cols());
                Eigen::VectorXd l_adj, u_adj;
                l_adj.resize(0); // Empty vector
                u_adj.resize(0); // Empty vector

                osqp::OsqpInstance instance;
                instance.objective_matrix = std::move(P);
                instance.objective_vector = que;
                instance.constraint_matrix = std::move(A_cons);
                instance.lower_bounds = l_adj;
                instance.upper_bounds = u_adj;

                osqp::OsqpSettings settings;
                settings.verbose = false;

                osqp::OsqpSolver solver;
                absl::Status st = solver.Init(instance, settings);

                Eigen::VectorXd delta_qr;
                osqp::OsqpExitCode exitcode = solver.Solve();
                if (exitcode != osqp::OsqpExitCode::kOptimal &&
                    exitcode != osqp::OsqpExitCode::kOptimalInaccurate)
                {
                    delta_qr = Eigen::VectorXd::Zero(nu); // fallback
                    // RCLCPP_WARN(get_node()->get_logger(), "OSQP solver failed! Exit code: %d", static_cast<int>(exitcode));
                }
                else
                {
                    Eigen::VectorXd sol = solver.primal_solution();
                    delta_qr = sol.segment(0, nu);
                }

                // // // até linha 555 ultimas modificações de  21/11

                // L = Eigen::MatrixXd::Zero(12, 38);
                // L.block(0, 6, 12, 12) = -kp * Eigen::MatrixXd::Identity(12, 12);
                // L.block(0, 26, 12, 12) = kp * Eigen::MatrixXd::Identity(12, 12);

                // // Publicar dqr no tópico do controlador de juntas
                auto low_Cmd = lowCmd();
                for (int j = 0; j < 12; ++j)
                {

                    qr[j] = qr[j] + delta_qr[j];
                    low_Cmd.motor_cmd[j].q = qr[j];
                    low_Cmd.motor_cmd[j].dq = 0;
                    low_Cmd.motor_cmd[j].kp = 50;
                    low_Cmd.motor_cmd[j].kd = 2.5;
                }

                joints_cmd_publisher_->publish(low_Cmd);
            }
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
        mat << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return mat;
    }

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> Go2RGC::define_constraints_matrices()
    {
        Eigen::MatrixXd aux_cons(n_c, n_j); // exemplo
        Eigen::MatrixXd Phi_cons(n_c * N, n_x + n_j);

        aux_cons.setZero();
        Phi_cons.setZero();

        return std::make_tuple(aux_cons, Phi_cons);
    }

    Eigen::Matrix<double, 4, 3> Go2RGC::rpy2Q(const Eigen::VectorXd &q)
    {
        Eigen::Matrix<double, 4, 3> T;
        T << q[3], q[2], -q[1],
            -q[2], q[3], q[0],
            q[1], -q[0], q[3],
            -q[0], -q[1], -q[2];
        return 0.5 * T;
    }

    // Eigen::MatrixXd Cf_fl, Cf_fr, Cf_rl, Cf_rr, Cf;
    // Eigen::MatrixXd Fc_mtx;
    Eigen::MatrixXd Go2RGC::cf_matrix(const Eigen::Vector3d &n, const Eigen::Vector3d &t1, const Eigen::Vector3d &t2, double mu)
    {
        Eigen::MatrixXd Cf(5, 3);
        Cf.row(0) = -mu * n + t1;
        Cf.row(1) = -mu * n + t2;
        Cf.row(2) = mu * n + t2;
        Cf.row(3) = mu * n + t1;
        Cf.row(4) = n;
        return Cf;
    }

    // Eigen::VectorXd Go2RGC::solve_rgc_osqp(
    //     const Eigen::MatrixXd &Phi,
    //     const Eigen::MatrixXd &G,
    //     const Eigen::MatrixXd &Phi_cons,
    //     const Eigen::MatrixXd &G_cons,
    //     const Eigen::VectorXd &x,
    //     const Eigen::VectorXd &ref,
    //     const Eigen::MatrixXd &Q,
    //     const Eigen::MatrixXd &R,
    //     const Eigen::VectorXd &l,
    //     const Eigen::VectorXd &u
    // )
    // {
    //     // Build cost
    //     Eigen::MatrixXd H_dense = G.transpose() * Q * G + R;

    //     // q = 2 * G^T * Q * (Phi*x - ref)
    //     Eigen::VectorXd diff = (Phi * x - ref);
    //     Eigen::VectorXd q = 2.0 * (G.transpose() * (Q * diff));

    //     Eigen::MatrixXd H_final = 2.0 * H_dense;

    //     // Convert to sparse (CSC)
    //     Eigen::SparseMatrix<double> P = H_final.sparseView();
    //     Eigen::SparseMatrix<double> A_cons = G_cons.sparseView();

    //     // Adjust bounds
    //     Eigen::VectorXd l_adj = l - (Phi_cons * x);
    //     Eigen::VectorXd u_adj = u - (Phi_cons * x);

    //     osqp::OsqpInstance instance;
    //     instance.objective_matrix = std::move(P);
    //     instance.objective_vector = q;
    //     instance.constraint_matrix = std::move(A_cons);
    //     instance.lower_bounds = l_adj;
    //     instance.upper_bounds = u_adj;

    //     osqp::OsqpSettings settings;
    //     settings.verbose = false;

    //     osqp::OsqpSolver solver;
    //     absl::Status st = solver.Init(instance, settings);
    //     if (!st.ok())
    //     {
    //         // Init failed
    //         return Eigen::VectorXd::Zero(nu);
    //     }

    //     osqp::OsqpExitCode exitcode = solver.Solve();
    //     if (exitcode != osqp::OsqpExitCode::kOptimal && exitcode != osqp::OsqpExitCode::kOptimalInaccurate)
    //     {
    //         return Eigen::VectorXd::Zero(nu);
    //     }

    //     // Get primal solution (returns Eigen::Map)
    //     Eigen::VectorXd sol = solver.primal_solution();
    //     if (static_cast<int>(sol.size()) < nu)
    //     {
    //         return Eigen::VectorXd::Zero(nu);
    //     }

    //     return sol.segment(0, nu);
    // }

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
