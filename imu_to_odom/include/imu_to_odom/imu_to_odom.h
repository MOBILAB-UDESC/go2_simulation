#ifndef IMU_TO_ODOM_IMU_TO_ODOM_H_
#define IMU_TO_ODOM_IMU_TO_ODOM_H_ 

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <eigen3/Eigen/Dense>


// #include <geometry_msgs/TransformStamped.h>
// #include <kindr/minimal/quat-transformation.h>
// #include <minkindr_conversions/kindr_msg.h>
// #include <nav_msgs/Odometry.h>
// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <tf/transform_broadcaster.h>
// #include <list>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h>

// typedef kindr::minimal::QuatTransformation Transformation;
// typedef kindr::minimal::RotationQuaternion Rotation;
// typedef Transformation::Vector3 Vector3;

class OdomPredictor : public rclcpp::Node
{
public:

  OdomPredictor();
 
  void lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg);


 private:
  
  void integrateIMUData(const unitree_go::msg::LowState::SharedPtr msg);

  void publishOdometry();
  void publishTransform();

  // void publishTF();

  bool has_imu_meas;
  // bool have_odom_;

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr imu_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  // ros::Publisher transform_pub_;

  // tf::TransformBroadcaster br_;

  int max_imu_queue_length_;

  std::list<unitree_go::msg::LowState> states_queue_;

  int seq_;
  std::string frame_id_;
  std::string child_frame_id_;

  uint32_t estimate_timestamp_;
  // Transformation transform_;
  Eigen::Affine3d transform_;
  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d angular_velocity_;

  Eigen::Vector3d imu_linear_acceleration_bias_;
  Eigen::Vector3d imu_angular_velocity_bias_;

  // boost::array<double, 36ul> pose_covariance_;
  // boost::array<double, 36ul> twist_covariance_;

  // Rotation orientation_;
  bool have_orientation_;


};

#endif  // IMU_TO_ODOM_IMU_TO_ODOM_H_
