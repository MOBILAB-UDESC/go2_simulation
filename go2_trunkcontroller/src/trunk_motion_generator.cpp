#include "trunk_motion_generator.hpp"

TrunkMotionGenerator::TrunkMotionGenerator(const pinocchio::Model& _robotModel) 
: config(YAML::LoadFile("config/go2.yaml"))
, robotModel(_robotModel)
, robotData(_robotModel)
, active(false)
, start(false)
, stopping(false)
, fCoeff(0.003)
, desired_height(0.35)
, traj(0.05, 5,  1, desired_height, 500)
{}

void TrunkMotionGenerator::init(
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
)
{
    starting_pose.set(Eigen::Vector3d(current_pose.toPosition()(0), current_pose.toPosition()(1), desired_height),
                        math::rpyToquat(Eigen::Vector3d(0.0, 0.0, current_pose.toRpy()(2))));
    desired_pose = starting_pose;

    traj.des_pos_offset_axes.block<3,1>(0,0) = starting_pose.toPosition();
    traj.des_pos_offset_axes.block<3,1>(3,0) = starting_pose.toRpy();
    traj.des_angle_axes = Eigen::Matrix<double, 6, 1>::Zero();

    base_pose_HF.set(traj.des_pos_axes.block<3,1>(0,0));
    base_pose_HF.set(math::rpyToquat(traj.des_pos_axes.block<3,1>(3,0)));
    base_velocity_HF.setLinear(traj.des_vel_axes.block<3,1>(0,0));
    base_velocity_HF.setAngular(traj.des_vel_axes.block<3,1>(3,0));
}

void TrunkMotionGenerator::stopMotion()
{
    // Smoothing desired pose and velocities
    desired_pose.set((1 - fCoeff) * desired_pose.toPosition() + fCoeff * starting_pose.toPosition());
    desired_pose.set(math::rpyToquat((1 - fCoeff) * desired_pose.toRpy() + fCoeff * starting_pose.toRpy()));
    desired_vel.setLinear((1 - fCoeff) * desired_vel.getLinear() + fCoeff * Eigen::Vector3d::Zero());
    desired_vel.setAngular((1 - fCoeff) * desired_vel.getAngular() + fCoeff * Eigen::Vector3d::Zero());

    if((desired_pose.toPosition() - starting_pose.toPosition()).norm() < 0.001 &&
        (desired_pose.toRpy() - starting_pose.toRpy()).norm() < 0.001)
    {
        this->stopping = false;
        std::cout << "Trunk motion generation stopped." << std::endl;
    }
}


void TrunkMotionGenerator::computeDesiredTrajectory(Pose& desired_pose, Screw& desired_vel)
{
    traj.des_pos_offset_axes(2) = starting_pose.toPosition()(2);
    traj.computeTrajectory();
    desired_pose.set(Eigen::Vector3d(traj.des_pos_axes(0), traj.des_pos_axes(1), traj.des_pos_axes(2)), math::rpyToquat(Eigen::Vector3d(traj.des_pos_axes(3), traj.des_pos_axes(4), traj.des_pos_axes(5))));
    desired_vel.setLinear(Eigen::Vector3d(traj.des_vel_axes(0), traj.des_vel_axes(1), traj.des_vel_axes(2)));
    desired_vel.setAngular(Eigen::Vector3d(traj.des_vel_axes(3), traj.des_vel_axes(4), traj.des_vel_axes(5)));
}


void TrunkMotionGenerator::run(
    const utils::Pose& input_current_pose,
    utils::Pose& base_pose_HF,
    utils::Screw& base_velocity_HF
)
{ 
    this->current_pose = input_current_pose;

    if(this->active)
    {
        if(this->start)
        {
            computeDesiredTrajectory(desired_pose, desired_vel);
        }
        else if(this->stopping)
        {
            stopMotion();
        }
        else
        {
            desired_pose.set((1-fCoeff) * desired_pose.toPosition() + fCoeff * desired_pose.toPosition());
        }

        base_pose_HF.set(math::rpyToRot(Eigen::Vector3d(0.0, 0.0, input_current_pose.toRpy()[2])) * desired_pose.toPosition());
        base_pose_HF.set(desired_pose.toQuaternion());
        base_velocity_HF.setLinear(desired_vel.getLinear());
        base_velocity_HF.setAngular(desired_vel.getAngular());
    }
}