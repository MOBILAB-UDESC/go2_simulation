#pragma once

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <vector>
#include <string>

namespace go2_rgc {

class RGCModel {
public:
  /**
   * @brief Constructor for the RGC model.
   * @param urdf_path Path to the URDF file of the robot.
   * @param control_horizon Prediction horizon N (e.g. 10 steps).
   * @param contact_frames List of frame names used as contacts (e.g., feet).
   */
  RGCModel(const std::string& urdf_path, 
           size_t control_horizon = 10,
           const std::vector<std::string>& contact_frames = {});

  /**
   * @brief Updates the robot state with joint positions and velocities.
   * @param q Joint positions (size nq)
   * @param v Joint velocities (size nv)
   * @param r Center of mass position in world frame
   * @param Q Base orientation as a quaternion
   */
  void updateState(const Eigen::VectorXd& q,
                   const Eigen::VectorXd& v,
                   const Eigen::Vector3d& r,
                   const Eigen::Quaterniond& Q);

    void computeLinearizedModel();

  /**
   * @brief Computes centroidal dynamics (Ag, hg) and linearizes A, b.
   */
  void computeCentroidalDynamics();

  /**
   * @brief Updates the contact Jacobians for the current joint configuration.
   */
  void updateContactJacobians();

  /**
   * @brief Builds the prediction model (Gx, Phix, Phig) based on current dynamics.
   * @param dt Control time step.
   */
  void buildPredictionModel(double dt);

  /**
   * @brief Computes gravity compensation torques (τ_g) via inverse dynamics.
   * NOTE: Not const because it modifies Pinocchio internal data.
   */
  Eigen::VectorXd computeGravityCompensation();

  /**
   * @brief Checks if the contact force is within the friction cone.
   * @param force 3D force vector at contact point.
   * @param mu Friction coefficient.
   */
  bool checkContactStability(const Eigen::Vector3d& force, double mu) const;

  // === Getters ===
  Eigen::Matrix<double, 4, 3> rpy2Q(const Eigen::Quaterniond& Q);
  // const Eigen::MatrixXd& getGx() const { return G_x_; }
  // const Eigen::MatrixXd& getPhiX() const { return Phi_x_; }
  // const Eigen::VectorXd& getPhiG() const { return Phi_g_; }
  // const std::vector<Eigen::MatrixXd>& getContactJacobians() const { return contact_jacobians_; }

private:
  // Pinocchio model and internal data
  pinocchio::Model model_;
  pinocchio::Data data_;

  // Prediction horizon
  size_t horizon_;

  // Frame names for contact points (e.g., feet)
  std::vector<std::string> contact_frames_;

  // Robot state
  Eigen::VectorXd q_;       // Joint positions
  Eigen::VectorXd v_;       // Joint velocities
  Eigen::Vector3d r_;       // Center of mass
  Eigen::Quaterniond Q_;    // Base orientation

  // Linearized centroidal dynamics: dx = A x + b
  Eigen::MatrixXd A_;
  Eigen::VectorXd B_;
  Eigen::VectorXd T_epsilon;
  

  // Prediction model matrices (discrete time)
  Eigen::MatrixXd Phi_x_;   // State transition matrix
  Eigen::VectorXd Phi_g_;   // Disturbance/gravity vector
  Eigen::MatrixXd G_x_;     // Toeplitz matrix (horizon x inputs)

  // Contact Jacobians for all specified frames
  // std::vector<Eigen::MatrixXd> contact_jacobians_;


};

} // namespace go2_rgc
