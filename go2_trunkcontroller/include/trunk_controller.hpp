#ifndef TRUNK_CONTROLLER_HPP
#define TRUNK_CONTROLLER_HPP

// #include <fstream>
#include "yaml-cpp/yaml.h"

// add other includes here
#include <Eigen/Dense>
#include "utils/pose.hpp"
#include "utils/screw.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

// #include <dls/modules/terrain_estimator.hpp>

// #include "controllers/trunk_controller/whole_body_optimizer.h"


namespace Eigen
{
    typedef Matrix<double, 6, 1> Vector6d;
}

//! Enum for coordinate definition
enum Coords6D { AX=0, AY, AZ, LX, LY, LZ };
class TrunkController
{
public:
    /*!
        * @brief Constructor
        * @param[in] pRobot shared pointer pointing to the robot object
        */
    TrunkController(const pinocchio::Model& robotModel);

    /*!
    * @brief Destructor
    */
    ~TrunkController();

    /*!
    * @brief Init function
    * @param[in] desired_height desired robot height
    */
    void init(double desired_height);

    /*!
    * @brief Run function
    */
    void run(
        const utils::Pose& pose
        , const utils::Screw& vel
        // const robotlib::JointState& q,
        // const robotlib::JointState& qd,
        // const robotlib::JointState& qdd,
        // const robotlib::JointState& tau,
        // const robotlib::LimbDataMap<bool> stance_status,
        // double& normal_force_min,
        // double& normal_force_max,
        , utils::Pose& des_pose_HF
        , const utils::Screw& des_vel_HF
        // const robotlib::LimbDataMap<bool> des_stance_status,
        // const robotlib::JointState& desQ,
        // const Eigen::Matrix<double, 6, 1> desired_wrench_W,
        // Eigen::Map<robotlib::JointState>& output_tau,
        // robotlib::LimbDataMap<Eigen::Vector3d>& output_curr_feet_forces,
        // robotlib::LimbDataMap<Eigen::Vector3d>& output_des_feet_forces,
        // Eigen::Map<Eigen::Vector6d>& output_des_wrench_W,
        // robotlib::LimbDataMap<double>& friction_coeff,
        // Eigen::Map<robotlib::JointState>& gravity_term,
        // Eigen::Map<robotlib::JointState>& coriolis_centrifugal_terms
    );

    /*!
        * @brief Compute the desired wrench in world frame
        * @param[in] pose robot pose
        * @param[in] des_pose desired robot pose
        * @param[in] vel robot velocity
        * @param[in] des_vel desired robot velocity
        * @return desired wrench
    */
    Eigen::Vector6d computeDesiredWrench(
        const utils::Pose& pose, 
        const utils::Pose& des_pose, 
        const utils::Screw& vel, 
        const utils::Screw& des_vel
    );

    /*!
        * @brief Map the desired wrench in joints torques
    */
    void computeOptWrenchToJointMapping(
        const utils::Pose& pose
        // const robotlib::JointState& q, 
        // const robotlib::LimbDataMap<bool>& des_stance_legs, 
        // const robotlib::LimbDataMap<double>& normal_force_min, 
        // const robotlib::LimbDataMap<double>& normal_force_max, 
        // const Eigen::Vector6d& desired_wrench_W, 
        // robotlib::JointState& joint_torques
    );

    /*!
        * @brief Get kp gain of a specific axis
    */
    Eigen::Vector6d getKp();
    /*!
        * @brief Get kp gain of a specific axis
    */
    Eigen::Vector6d getKd();

    /*!
        * @brief Get kp gain of a specific axis
        * @param[in] axis
    */
    double getKp(int axis);

    /*!
        * @brief Get kd gain of a specific axis
        * @param[in] axis
    */
    double getKd(int axis);

    /*!
        * @brief Set linear and angular kp gain
        * @param[in] gain kp gain
    */
    void setKp(double gain);

    /*!
        * @brief Set linear and angular kd gain
        * @param[in] gain kd gain
    */
    void setKd(double gain);

    /*!
        * @brief Set kp gain of a specific axis
        * @param[in] gain kp gain
        * @param[in] axis axis to which apply the gain
    */
    void setKp(double gain, int axis);

    /*!
        * @brief Set kd gain of a specific axis
        * @param[in] gain kd gain
        * @param[in] axis axis to which apply the gain
    */
    void setKd(double gain, int axis);

    /*!
        * @brief Switch to next mode
    */
    void switchMode();

    bool use_input_desired_wrench;
    bool use_computed_wrench;

private:
    //! YAML node
    YAML::Node config;

    //! Robot object
    const pinocchio::Model& robotModel;

    //! Robot data
    pinocchio::Data robotData;

    //! Whole body optimization
    // std::shared_ptr<WholeBodyOptimizer> _whole_body;

    //! Terrain Estimator
    // TerrainEstimator terrain_estimator;

    //! Actual foot position
    // robotlib::LimbDataMap<Eigen::Vector3d> actual_foot_position;

    //! Coriolis, Centrifugal and Gravitational terms
    // robotlib::JointState h_joints;

    //! Friction coefficients
    // robotlib::LimbDataMap<double> friction_coefficients;

    //! Gravity acceleration value
    const double g;

    //! Gravity acceleration vector in base frame
    Eigen::Vector6d g_b;

    //! Desired filtered height
    double desired_height_filtered;

    //! Gravity term for debugging
    // robotlib::JointState output_gravity_term;

    //! Kp gains
    Eigen::Vector6d kp;
    //! Kd gains
    Eigen::Vector6d kd;

    //! Mode
    int _mode;

    //! Wrench offset
    Eigen::Vector6d wrench_offset;

    //! Robot proprio height
    double proprio_height;

    //! Variable for activating position adjustment based on estimated terrain inclination
    bool terr_estim_post_adj;

    //! Variable for using the terrain surface normal
    bool terr_estim_friction_cones;

    // std::ofstream _log;

    //! debug variables
    Eigen::Matrix<double, 36, 1> _dbg_vec;
};


#endif /* end of include guard: TRUNK_CONTROLLER_HPP */
