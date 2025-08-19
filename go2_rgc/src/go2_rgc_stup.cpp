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