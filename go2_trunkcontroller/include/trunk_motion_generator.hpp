#ifndef TRUNK_MOTION_GENERATOR_HPP
#define TRUNK_MOTION_GENERATOR_HPP

#include <iostream>
#include "yaml-cpp/yaml.h"

#include "utils/pose.hpp"
#include "utils/screw.hpp"
#include "math/rotations.hpp"

#include "sine_wave_trajectory.hpp"

// #include <dls2/msg_wrappers/pose.hpp>
// #include <dls2/msg_wrappers/screw.hpp>


// add other includes here

class TrunkMotionGenerator
{
public:
    /*!
    * @brief Constructor
    */
    TrunkMotionGenerator();

    /*!
    * @brief Destructor
    */
    ~TrunkMotionGenerator();

    void init(
        // const robotlib::JointState& input_joints_position,
        utils::Pose& base_pose_HF,
        utils::Screw& base_velocity_HF,
        // robotlib::JointState& output_desired_joints_position,
        // robotlib::JointState& output_desired_joints_velocity,
        // robotlib::JointState& output_desired_joints_acceleration,
        // robotlib::JointState& output_desired_joints_effort,
        utils::Pose& output_desired_com_pose_world
        // robotlib::LegDataMap<bool>& stance_legs,
        // robotlib::LegDataMap<Eigen::Vector3d>& nominal_touch_down,
        // robotlib::LegDataMap<Eigen::Vector3d>& touch_down,
        // robotlib::LegDataMap<double>& swing_period
    );  

    /*!
    * @brief Run function
    */
    void run(/*input_arguments, output_arguments*/);

private:
    YAML::Node config;

    //! Robot object
    const pinocchio::Model& robotModel;

    //! Robot data
    pinocchio::Data robotData;
    
    bool active;
    bool start;
    bool stopping;

    double fCoeff;
    double desired_height;

    utils::Pose starting_pose;
    utils::Pose current_pose;
    utils::Pose desired_pose;
    utils::Screw desired_vel;

    SineWaveTrajectory traj;

    // robotlib::LegDataMap<Eigen::Vector3d> m_des_feet_pos;
    // robotlib::LegDataMap<Eigen::Vector3d> m_des_feet_vel;

    // robotlib::LegDataMap<Eigen::Vector3d> actual_foot_pos_HF;

    // robotlib::LegDataMap<Eigen::Vector3d> feet_home_configuration;
    // robotlib::LegDataMap<Eigen::Vector3d> feet_position;

    // robotlib::JointState home_configuration;
    // robotlib::JointState fold_configuration;
};

#endif /* end of include guard: TRUNK_MOTION_GENERATOR_HPP */
