#include "trunk_controller.hpp"

#include <iostream>
#include "pinocchio/algorithm/rnea.hpp"
// #include <cmath>
// #include <cstdlib>
// #include <dls2/math/rotations.hpp>
// #include <robotlib/utils/utils.hpp>


TrunkController::TrunkController(const pinocchio::Model& _robotModel)
: use_input_desired_wrench(true)
, use_computed_wrench(true)
, config(YAML::LoadFile("config/config.yaml")["go2"])
, robotModel(_robotModel)
, robotData(_robotModel)
// , terrain_estimator(pRobot)
// , actual_foot_position(pRobot->makeLimbDataMap<Eigen::Vector3d>(Eigen::Vector3d::Zero()))
// , h_joints(pRobot->makeJointState(0.0))
// , friction_coefficients(pRobot->makeLimbDataMap<double>(0.0))
, g(9.81)
, desired_height_filtered(0.0)
, g_b(Eigen::Vector6d::Zero())
// , output_gravity_term(pRobot->makeJointState(0.0))
, kp(config["kp"].as<std::vector<double>>().data())
, kd(config["kd"].as<std::vector<double>>().data())
, wrench_offset(0, 0, 0, 0, 0, 0)
, proprio_height(config["proprio_height"].as<double>())
, terr_estim_post_adj(config["terr_estim_post_adj"].as<bool>())
, terr_estim_friction_cones(config["terr_estim_friction_cones"].as<bool>())
, _mode(0)
{
    // Set friction coefficient
    // for(auto leg : pRobot->getLegs())
    // {
    //     friction_coefficients[leg] =
    //         config["friction_coefficients"][leg->getName()].as<double>();
    // }

    _dbg_vec.setZero();
}

TrunkController::~TrunkController()
{
    std::cout << "[TRNK_CTRL] destruction!" << std::endl;
}

void TrunkController::init(double desired_height)
{
    desired_height_filtered = desired_height;

    // New dls2 approach:
    // if (!_whole_body.get())
    //   _whole_body.reset(new WholeBodyOptimizer(pRobot));

    std::cout << "[TRNK_CTRL] init() called!\n params from config: " << std::endl;
    std::cout << "  kp: " << kp.transpose() << std::endl;
    std::cout << "  kd: " << kd.transpose() << std::endl;
    std::cout << "  proprio_height: " << proprio_height << std::endl;
    std::cout << "  terr_estim_post_adj: " << terr_estim_post_adj << std::endl;
    std::cout << "  terr_estim_friction_cones: " << terr_estim_friction_cones << std::endl;
}


void TrunkController::run(
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
    // Eigen::Map<robotlib::JointState>& output_gravity_term,
    // Eigen::Map<robotlib::JointState>& output_coriolis_centrifugal_terms
)
{
    // static utils::Pose straightPose;
    // straightPose.set(pose.toPosition(), pose.toQuaternion());

    // This trunk controller can only track controller command references, not desired com trajectory
    // this->g_b = robotlib::utils::computeGravity_B(straightPose.toRotationMatrix(), this->g);
    // this->pRobot->forwardKinematics(q, this->actual_foot_position);
    // pRobot->computeProprioHeight(straightPose.toRpy(), stance_status, this->actual_foot_position, this->proprio_height);

    // static robotlib::LimbDataMap<bool> cmbStnc(des_stance_status);
    // for (auto leg : pRobot->getLegs())
    //   cmbStnc[leg] = des_stance_status.at(leg);// && stance_status[leg];
    // if (_mode % 4 == 1) cmbStnc.at(pRobot->getLegs()[0]) = false;
    // if (_mode % 4 == 3) cmbStnc.at(pRobot->getLegs()[1]) = false;
    // if (_mode % 4 == 1) cmbStnc.at(pRobot->getLegs()[3]) = false;
    // if (_mode % 4 == 3) cmbStnc.at(pRobot->getLegs()[2]) = false;

    static Eigen::Matrix<double, 6, 1> computedWrench(0, 0, 0, 0, 0, 0);
    computedWrench = computeDesiredWrench(pose, des_pose_HF, vel, des_vel_HF);

    // // TODO: Readd desired wrench from user input in the future
    // // output_des_wrench_W = wrench_offset;
    // // if (use_computed_wrench) output_des_wrench_W += computedWrench;
    // // if (use_input_desired_wrench) output_des_wrench_W += desired_wrench_W;

    // pRobot->computeNonLinearEffects(straightPose.toVector(),
    //                                 q, qd, h_joints);

    // static Eigen::Vector3d vec_surf_normal(0.0, 0.0, 1.0);

    // // new dls2 approach:
    // static robotlib::LimbDataMap<Eigen::Vector3d> surfNormalMap(
    //     pRobot->makeLimbDataMap<Eigen::Vector3d>(vec_surf_normal));

    // friction_coeff = this->friction_coefficients;

    // static Eigen::Vector3d comShift(Eigen::Vector3d::Zero());

    // _whole_body->computeOptimizationWithArm(
    //     surfNormalMap,
    //     friction_coeff,
    //     cmbStnc,
    //     normal_force_max,
    //     normal_force_min,
    //     straightPose,
    //     vel,
    //     q,
    //     qd,
    //     desQ,
    //     computedWrench, // output_des_wrench_W,
    //     comShift);

    // output_tau << _whole_body->getJointTorques().array().isNaN().select(19.04, _whole_body->getJointTorques());
    // _dbg_vec << output_tau;

    // output_des_feet_forces = _whole_body->getFeetForces();
}

Eigen::Vector6d TrunkController::computeDesiredWrench(
    const utils::Pose& pose, 
    const utils::Pose& des_pose, 
    const utils::Screw& vel, 
    const utils::Screw& des_vel
)
{
    Eigen::Vector6d desired_wrench = Eigen::Vector6d::Zero();
    Eigen::Vector3d des_lin_vel_HF = des_vel.getLinear();
    Eigen::Vector3d des_ang_vel_HF = des_vel.getAngular();
    Eigen::Vector3d lin_vel_world = vel.getLinear();
    Eigen::Vector3d ang_vel_world = vel.getAngular();

    Eigen::Quaterniond pose_quat = pose.toQuaternion();
    Eigen::Matrix3d w_R_base = pose_quat.toRotationMatrix();
    Eigen::Matrix3d base_R_w = w_R_base.transpose();

    Eigen::Quaterniond des_pose_quat = des_pose.toQuaternion();
    Eigen::Vector3d w_rpy_b (pose.toRpy());

    Eigen::AngleAxisd yawAngle(w_rpy_b[2], Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d w_R_horizontal = yawAngle.toRotationMatrix();

    Eigen::Matrix3d des_R = des_pose_quat.toRotationMatrix();

    Eigen::Vector3d des_lin_vel_world = w_R_horizontal * des_lin_vel_HF;
    Eigen::Vector3d des_ang_vel_world = w_R_horizontal * des_ang_vel_HF;

    // Assuming standard X-Y-Z convention in eigen [roll, pitch, yaw]
    Eigen::Vector3d des_rpy = des_R.eulerAngles(0, 1, 2);
    double pitch = des_rpy[1];

    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());

    // Combine yaw from base, pitch from desired, roll = 0. R = Rz(yaw) * Ry(pitch)
    Eigen::Matrix3d R_combined = (yawAngle * pitchAngle).toRotationMatrix();
    Eigen::Quaterniond target_quaternion(R_combined);

    Eigen::Quaterniond actual_quaternion = pose_quat;

    // Compute the orientation delta
    Eigen::Quaterniond delta_quaternion = target_quaternion * actual_quaternion.inverse();
    Eigen::AngleAxisd delta_angle_axis(delta_quaternion);
    Eigen::Vector3d orientation_delta = delta_angle_axis.axis() * delta_angle_axis.angle();

    desired_wrench(AX) = kp(AX) * orientation_delta[0] - (kd(0) * ang_vel_world(0));
    desired_wrench(AY) = kp(AY) * orientation_delta[1] - (kd(1) * ang_vel_world(1));
    desired_wrench(AZ) = kd(AZ) * (des_ang_vel_world(2) - ang_vel_world(2));
    desired_wrench(LX) = kd(LX) * (des_lin_vel_world(0) - vel.getLinear()(0));
    desired_wrench(LY) = kd(LY) * (des_lin_vel_world(1) - vel.getLinear()(1));

    // TODO: Fix here. No hardcoded desired height
    desired_wrench(LZ) = kp(LZ) * (0.366 - this->proprio_height) - kd(5) * vel.getLinear()(2);

    // Transform wrench from world to base frame. The application point of the wrench is the same so the cross term is not needed.
    Eigen::Matrix<double,6,6> AdT = Eigen::Matrix<double,6,6>::Zero();
    AdT.topLeftCorner<3,3>()     = base_R_w;
    AdT.bottomRightCorner<3,3>() = base_R_w;

    Eigen::Vector6d desired_wrench_base = AdT * desired_wrench;

    return desired_wrench_base;
}

Eigen::Vector6d TrunkController::getKp()
{
    return this->kp;
}

Eigen::Vector6d TrunkController::getKd()
{
    return this->kd;
}

double TrunkController::getKp(int axis)
{
    return this->kp[axis];
}

double TrunkController::getKd(int axis)
{
    return this->kd[axis];
}

void TrunkController::setKp(double gain)
{
    this->kp.array() = gain;
}

void TrunkController::setKd(double gain)
{
    this->kd.array() = gain;
}

void TrunkController::setKp(double gain, int axis)
{
    this->kp[axis] = gain;
}

void TrunkController::setKd(double gain, int axis)
{
    this->kd[axis] = gain;
}

void TrunkController::switchMode()
{
    _mode = (_mode + 1) % 8;
}