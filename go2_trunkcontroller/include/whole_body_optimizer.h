/**
  * whole_body_optimizer.h
  *
  * Created on: 2025-03-04
  *     Author: stittel
  *   Based on: WholeBodyOptimizationWithArm from DLS1 (by mrisiglione)
  */

#ifndef WHOLE_BODY_OPTIMIZER_H_
#define WHOLE_BODY_OPTIMIZER_H_

#include <memory>
#define M_PI 3.14159265358979323846 /* pi */

#include <thread>

#include <Eigen/Dense>

#include <gtk/gtk.h>

#include <dls2/util/screw.hpp>
#include <dls2/util/pose.hpp>

#include <robotlib/robot_base.hpp>

namespace controllers
{
  /**
    * @class WholeBodyOptimizer
    * @brief A class for performing Whole-Body optimization for a quadruped robot with an integrated arm.
    *
    * This class provides the methods and data structures necessary for solving QP-based optimization problems
    * involving a robot's base, legs, and arm. It includes functionality for generating non-linear control
    * commands, computing dynamic and kinematic quantities, and handling constraints related to motion and interaction
    * tasks.
    *
    * @details The optimization is designed to handle tasks such as respecting physical consistency constraints, tracking desired
    * trajectories for each limb, and impedance rendering. It accounts for the coupling between the robot's base and arm through the
    * physical consistency constraint (base dynamics), and takes also into account torque constraints, friction constraints (to avoid
    * slippage), and joint limits.
    *
    * ### Key Features
    * - Computes full-body dynamics and kinematics within the cpp implementation.
    * - Supports optimization for both Cartesian and joint space tracking tasks for the non-locomotive arm.
    * - Handles stance and swing phase constraints for the legs.
    * - Implements impedance rendering for the arm's end-effector.
    * - Provides tools for singularity avoidance and debugging optimization results.
    * - Includes user-configurable parameters for task weighting and constraint activation.
    *
    * ### Main Methods
    * - **Optimization**: `computeOptimizationWithArm()` calculates the feedback-linearization based control commands.
    * - **Kinematics and Dynamics**: Methods to compute Jacobians, accelerations, and constraints.
    * - **Getters and Setters**: Access or modify task weights, gains, and constraints dynamically.
    * - **Debugging**: Tools to analyze optimization results and singularity measures.
    *
    * ### Usage
    * This class is intended for roboticists working with quadruped robots equipped with manipulator(s).
    * It provides a way to generate control inputs for whole-body control.
    *
    * @note Ensure that all necessary inputs, such as robot state, desired trajectories, and constraint
    * parameters, are provided before invoking the optimization routines.
    *
    * @author Mattia Risiglione
    * @date Created on May 21, 2021
    */
  class WholeBodyOptimizer
  {

  public:

    /**
      * @brief Constructor for WholeBodyOptimizer.
      */
    WholeBodyOptimizer(const std::shared_ptr<robotlib::RobotBase>& robot);

    /**
      * @brief Deconstructor for WholeBodyOptimizer.
      */
    ~WholeBodyOptimizer();

    /**
      * @brief Compute a dummy motion without respecting gravity.
      */
    void computeDummyMotionInSpace();

    /**
      * @brief Compute the whole-body optimization with an arm.
      */
    void computeOptimizationWithArm(
        const robotlib::LimbDataMap<Eigen::Vector3d>& surfNormal,
        const robotlib::LimbDataMap<double>& muEstimate,
        const robotlib::LimbDataMap<bool>& stanceLegs,
        const double& forceMax,
        const double& forceMin,
        const dls::utils::Pose& basePose,
        const dls::utils::Screw& baseTwist,
        const robotlib::JointState& q,
        const robotlib::JointState& qd,
        const robotlib::JointState& desQ,
        const robotlib::Vec6d& desWrench,
        const Eigen::Vector3d& comShift);

    //! number of active joints (arm + legs)
    static const int DOF_JOINTS_ALL = 19;

    //! Number of arm joints
    static const int DOF_JOINTS_ARM = 7;

    //! number of base dofs
    static const int DOF_CART_BASE = 6;

    //! number of end effector dofs
    static const int DOF_CART_EE = 6;

    // Lists of getters

    //! Retrieve feet forces
    robotlib::LimbDataMap<Eigen::Vector3d>& getFeetForces();

    //! Retrieve total joint torques computed via inverse dynamics
    robotlib::JointState& getJointTorques();

    //! Update current joint positions, velocities, and accelerations
    void updateJointState(const robotlib::JointState& positions,
                          const robotlib::JointState& velocities,
                          const robotlib::JointState& accelerations);

    //! Get vector of all pairs for debugging
    static std::vector<std::pair<std::string, int>> getDbgPairs();

    //! Fill "vec" with data coressponding to "name"
    void fillDbgVec(std::string& name, std::vector<double>& vec);

    void updateMarkers(GtkWidget* window);
    void getCom(Eigen::Vector3d& com);
    void getDesPos(Eigen::Vector3d& desPos);
    void getTrunkPos(Eigen::Vector3d& trunkPos);
    void getFeetCenter(Eigen::Vector3d& feetCenter);
    void getFeetPos(Eigen::Vector3d& lf, Eigen::Vector3d& rf,
                    Eigen::Vector3d& lh, Eigen::Vector3d& rh);

  protected:
    //! Method use to setup certain kinematic and dynamic quantities
    void prepareOptimization();

    //! Method to setup cost functions
    void setCostFunction();

    //! Method to fill equality matrices
    void setEqualities();

    //! Method to fill inequality matrices
    void setInequalities(const robotlib::JointState& desQ);

    //! Method to compute swing accelerations for legs
    void computeOperationalSpaceSwingtask(const robotlib::JointState& desQ);

    //! Method to compute desired joint accelerations to track arm joint references
    void computeArmTrackingTaskJointSpace();

    /**
      * @brief Computes the stance acceleration for a given leg.
      *
      * This method calculates the acceleration vector for a specific leg
      * during the stance phase of locomotion.
      *
      * @return Eigen::Vector3d indicating the computed acceleration vector in 3D space.
      */
    Eigen::Vector3d computeStanceAcceleration();

    void resetFeetRelativeDistances();

    bool detectStanceChange();

  private:
    //! Robot object
    std::shared_ptr<robotlib::RobotBase> _robot_ptr;

    // current robot state used as input
    robotlib::JointState _joint_positions;
    robotlib::JointState _joint_velocities;
    robotlib::JointState _joint_accelerations;
    dls::utils::Pose _base_pose;
    dls::utils::Screw _base_twist;
    robotlib::LimbDataMap<bool> _stance_legs;

    // resulting variables used as output
    robotlib::JointState _joint_torques;
    robotlib::LimbDataMap<Eigen::Vector3d> _feet_forces;
    robotlib::JointState _nle_joints;

    // matrices for arm tracking tasks; ToDo: check if needed
    Eigen::DiagonalMatrix<double, 3> _kp_ost_arm;
    Eigen::DiagonalMatrix<double, 3> _kd_ost_arm;
    Eigen::DiagonalMatrix<double, 3> _kp_oso_arm;
    Eigen::DiagonalMatrix<double, 3> _kd_oso_arm;
    Eigen::DiagonalMatrix<double, 7> _kp_posture_arm;
    Eigen::DiagonalMatrix<double, 7> _kd_posture_arm;

    // variables defining environment and robot properties
    robotlib::LimbDataMap<Eigen::Vector3d> _surf_normal;
    robotlib::LimbDataMap<double> _mu_estimate;
    double _force_max;
    double _force_min;

    // variables defining desired behavior or fallback references
    robotlib::LimbDataMap<Eigen::Vector3d> _des_swing_pos;
    robotlib::LimbDataMap<Eigen::Vector3d> _des_swing_vel;
    robotlib::LimbDataMap<Eigen::Vector3d> _des_swing_acc;
    robotlib::LimbDataMap<Eigen::Vector3d> _foot_pos_map;
    robotlib::LimbDataMap<Eigen::Matrix3d> _foot_rot_map;
    robotlib::LimbDataMap<Eigen::Vector3d> _foot_vel_map;
    robotlib::LimbDataMap<robotlib::Vec6d> _foot_twist_map; // may be merged with vel
    robotlib::LimbDataMap<robotlib::Vec6d> _foot_acc_map;
    robotlib::LimbDataMap<Eigen::Matrix3d> _foot_jac_map;
    Eigen::Matrix<double, 7, 1> _joints_arm_ref_pos;
    Eigen::Matrix<double, 7, 1> _joints_arm_ref_vel;
    Eigen::Matrix<double, 7, 1> _joints_arm_ref_acc;
    robotlib::Vec6d _wrench_des;
    robotlib::Vec6d _wrench_ee;
    robotlib::Vec6d _nle_base;
    double _wght_arm;
    double _wght_wrench;

    // variables for calculations
    Eigen::Matrix4d _arm_pose;
    Eigen::VectorXd _jnt_vel_eig;
    Eigen::Matrix3d _skew_mat;
    Eigen::Matrix3d _base_rot;
    Eigen::Matrix3d _base_rot_inv;
    Eigen::MatrixXd _foot_jac;
    Eigen::MatrixXd _inertia_js;
    Eigen::VectorXd _x;
    Eigen::MatrixXd _GQ;
    Eigen::VectorXd _g0;
    Eigen::MatrixXd _CE;
    Eigen::VectorXd _ce0;
    Eigen::MatrixXd _CI;
    Eigen::VectorXd _ci0;
    Eigen::VectorXd _cc_torques;
    robotlib::Vec6d _base_wrench;
    Eigen::MatrixXd _jac_stnc;
    Eigen::MatrixXd _jac_swng;
    Eigen::MatrixXd _tmp_jac;
    Eigen::VectorXd _jac_stnc_qd; // find better name by determining the actual meaning
    Eigen::VectorXd _jac_swng_qd; // find better name by determining the actual meaning
    Eigen::MatrixXd _arm_jac;
    Eigen::MatrixXd _A;
    Eigen::MatrixXd _A_arm;
    Eigen::MatrixXd _A_swing;
    Eigen::MatrixXd _B;
    Eigen::Matrix<double, 6, 1> _b;
    Eigen::Matrix<double, 7, 1> _b_arm;
    Eigen::VectorXd _b_swing;
    Eigen::MatrixXd _W;
    int _num_eq;
    int _num_ineq;
    int _num_slacks;
    int _cntct_frcs;
    int _cleg_cnt;
    int _swng_cnstr;
    int _fric_cnstr;

    //! Status flag
    bool _running;

    // drawing variables
    Eigen::Vector3d _des_pos;
    Eigen::Vector3d _com;
    Eigen::Vector3d _pos;
    Eigen::Matrix3d _rot;
    std::shared_ptr<std::thread> _marker_thread;

    //! feet center (intersection point of feet diagonals)
    Eigen::Vector3d _feet_center;
  };

}
#endif /* WHOLE_BODY_OPTIMIZER_H_ */
