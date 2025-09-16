#include <go2_rgc/go2_rgc.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/Geometry>

namespace go2_rgc {

RGCModel::RGCModel(const std::string& urdf_path,
                   size_t control_horizon,
                   const std::vector<std::string>& contact_frames)
    : horizon_(control_horizon), contact_frames_(contact_frames) {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    q_ = pinocchio::neutral(model_);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    r_ = Eigen::Vector3d::Zero();
    Q_ = Eigen::Quaterniond::Identity();
}

void RGCModel::updateState(const Eigen::VectorXd& q,
                           const Eigen::VectorXd& v,
                           const Eigen::Vector3d& r,
                           const Eigen::Quaterniond& Q) {
    q_ = q;
    v_ = v;
    r_ = r;
    Q_ = Q;
    pinocchio::forwardKinematics(model_, data_, q_, v_);
    pinocchio::updateFramePlacements(model_, data_);
    computeLinearizedModel();
}

void Go2RGC::computeLinearizedModel(const Eigen::VectorXd &q) {
    const int n_j = 7;  // Número de juntas ativas (ajuste conforme seu robô)
    const int n_x = 17; // Dimensão do estado: [r_dot (3), ω (3), q (7), r (3), ε (4)]
    
    // // 1. Jacobiano de contato concatenado (Jc)
    // Eigen::MatrixXd Jc(3 * 4, model.nv);
    // for (size_t i = 0; i < 4; ++i) {
    //     pinocchio::Data::Matrix6x J(6, model.nv);
    //     J.setZero();
    //     pinocchio::computeFrameJacobian(model, *data, _q, 
    //                                   model.getFrameId("1_FR_foot"), 
    //                                   pinocchio::LOCAL_WORLD_ALIGNED, J);
    //     Jc.block(3 * i, 0, 3, model.nv) = J.topRows<3>();
    // }

    Eigen::MatrixXd Jcom_full = pinocchio::jacobianCenterOfMass(model, *data, q);
        // const int num_contacts = 4;
    // Remove os 6 DoF da base → pega apenas as colunas das juntas
    Jcom = Jcom_full.block(0, 6, 3, 12); // 3 linhas (x,y,z), 12 colunas (juntas)

    // Jacobiano de contato (um bloco 3x12 por pé)
    const int num_contacts = 4;
    Jc.resize(3 * num_contacts, 12); // 12x12 no total
    Jc.setZero();

    // for (int i = 0; i < num_contacts; ++i)
    // {
    //     // ID do frame do pé (último frame de cada perna)
    //     int frame_id = _frame_index[i * 4 + 3];

    //     // Jacobiano 6x18 completo
    //     pinocchio::Data::Matrix6x Jframe(6, model.nv);
    //     Jframe.setZero();

    //     pinocchio::computeFrameJacobian(model, *data, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, Jframe);

    //     // Pegamos só as 3 primeiras linhas (linear) e as 12 colunas das juntas
    //     Eigen::MatrixXd J_leg = Jframe.topRows<3>().block(0, 6, 3, 12);

    //     // Inserimos na linha correspondente do Jc
    //     Jc.block(3 * i, 0, 3, 12) = J_leg;
    // }

    //// 2. Matrizes Γ₁* e Γₐ* (equação 14 do artigo)
   // Eigen::MatrixXd Gamma = Jc;  // Simplificado (Γ = Jc no artigo)
   // Eigen::MatrixXd Gamma_1_star = Gamma.block(0, 0, 3, n_j);  // Primeiras 3 linhas
    //Eigen::MatrixXd Gamma_a_star = Gamma.block(3, 0, 3, n_j);  // Próximas 3 linhas

    // Supondo que já temos: Jcom (3x12), Jc (12x12) e r_base (posição da base)

    // === Cálculo de Gamma ===
    // Eigen::MatrixXd Gamma(12, 12);
    // Gamma.block(0, 0, 3, 12) = Jcom;
    // Gamma.block(3, 0, 3, 12) = Jcom;
    // Gamma.block(6, 0, 3, 12) = Jcom;
    // Gamma.block(9, 0, 3, 12) = Jcom;
    // Gamma -= Jc;  // Gamma = [J_CoM; J_CoM; J_CoM; J_CoM] - Jc

//     // === Inversa de Gamma ===
//     Eigen::MatrixXd Gamma_inv = Gamma.inverse();
//     // === Blocos Gamma_1_star e Gamma_a_star ===
//     Eigen::MatrixXd Gamma_1_star = Gamma.block(0, 0, 3, 12);   // linhas 0-2
//     Eigen::MatrixXd Gamma_a_star = Gamma.block(3, 0, 3, 12);   // linhas 3-5


//     // === Cálculo de GAMMA_lin e GAMMA_ang ===
//     Eigen::MatrixXd GAMMA_lin = Eigen::MatrixXd::Zero(12, 3);
//     Eigen::MatrixXd GAMMA_ang = Eigen::MatrixXd::Zero(12, 3);

//     // === Criação da matriz S_gamma ===
//     // Assumindo que você já tem os frames dos pés em LOCAL_WORLD_ALIGNED
//     std::vector<Eigen::Vector3d> foot_positions;

//     for (int i = 0; i < 4; ++i) {
//         int frame_id = _frame_index[i * 4 + 3];  // índice do pé
//         const auto& foot_placement = data->oMf[frame_id];
//         foot_positions.push_back(foot_placement.translation());
//     }

//     Eigen::MatrixXd S_gamma(3, 12);
//     Eigen::Vector3d r_base(0, 0, 0); // centro de massa estimado

//     for (int i = 0; i < 4; ++i) {
//         Eigen::Matrix3d cross;
//         Eigen::Vector3d rel = foot_positions[i] - r_base;
//         cross <<      0, -rel.z(),  rel.y(),
//                 rel.z(),       0, -rel.x(),
//                 -rel.y(),  rel.x(),       0;
//         S_gamma.block(0, 3 * i, 3, 3) = cross;
//     }

//     // === Loop de soma GAMMA_lin e GAMMA_ang ===
//     for (int i = 0; i < 4; ++i) {
//         GAMMA_lin += Gamma_inv.block(0, 3 * i, 12, 3);
//         GAMMA_ang += Gamma_inv.block(0, 3 * i, 12, 3) * S_gamma.block(0, 3 * i, 3, 3);
//     }

//     // === Impressão ===
//     std::cout << "Gamma:\n" << Gamma << std::endl;
//     std::cout << "Gamma_inv:\n" << Gamma_inv << std::endl;
//     std::cout << "GAMMA_lin:\n" << GAMMA_lin << std::endl;
//     std::cout << "GAMMA_ang:\n" << GAMMA_ang << std::endl;


//     // 3. Matrizes K₁, K₂, K₃, K₄ (ganhos do controlador PD)
//     double Kp = 100.0, Kd = 10.0;
//     Eigen::MatrixXd K1 = Eigen::MatrixXd::Identity(3, n_j) * Kp;
//     Eigen::MatrixXd K2 = Eigen::MatrixXd::Identity(3, n_j) * Kd;
//     Eigen::MatrixXd K3 = Eigen::MatrixXd::Identity(3, n_j) * Kp;
//     Eigen::MatrixXd K4 = Eigen::MatrixXd::Identity(3, n_j) * Kd;

//     // 4. Matriz A (17x17)
//     A_.resize(n_x, n_x);
//     A_.setZero();

//     // Preenche blocos conforme a equação da imagem
//     A_.block(0, 0, 3, 3) = -K2 * Gamma_1_star;  // -K₂Γ₁*
//     A_.block(0, 3, 3, 3) = K2 * Gamma_a_star;    // K₂Γₐ*
//     A_.block(0, 6, 3, n_j) = -K1;                // -K₁

//     A_.block(3, 0, 3, 3) = -K4 * Gamma_1_star;   // -K₄Γ₁*
//     A_.block(3, 3, 3, 3) = K4 * Gamma_a_star;    // K₄Γₐ*
//     A_.block(3, 6, 3, n_j) = -K3;                // -K₃

//     A_.block(6, 0, n_j, 3) = Gamma_1_star;       // Γ₁*
//     A_.block(6, 3, n_j, 3) = -Gamma_a_star;      // -Γₐ*

//     A_.block(9, 0, 3, 3) = Eigen::Matrix3d::Identity();  // I (integra r_dot -> r)
//     A_.block(13, 3, 4, 3) = rpy2Q(Q_);              // T_ε (integra ω -> ε)

//     // 5. Matriz B (17x7) - Apenas B_u (K₁ e K₃)
//     B_.resize(n_x, n_j);
//     B_.setZero();
//     B_.block(0, 0, 3, n_j) = K1;  // K₁
//     B_.block(3, 0, 3, n_j) = K3;  // K₃
 }

// Helper: Matriz T_epsilon para quatérnios
Eigen::Matrix<double, 4, 3> RGCModel::rpy2Q(const Eigen::Quaterniond& Q) {
    Eigen::Matrix<double, 4, 3> T;
    T << -Q.x(), -Q.y(), -Q.z(),
          Q.w(), -Q.z(),  Q.y(),
          Q.z(),  Q.w(), -Q.x(),
         -Q.y(),  Q.x(),  Q.w();
    return 0.5 * T;
}

} // namespace go2_rgc