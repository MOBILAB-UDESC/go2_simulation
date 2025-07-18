#include "imu_to_odom/imu_to_odom.h"
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

  // CLASS INITIALIZATION
OdomPredictor::OdomPredictor()
  : Node("OdomPredictor")
  , seq_(0)
  , has_imu_meas(false) //FIRST INITIALIZATION OF IMU? 
  , imu_linear_acceleration_bias_(0, 0, 0)//CALIBRATION
  , imu_angular_velocity_bias_(0, 0, 0)
  , have_orientation_(true)//IMU ORIENTATION VALIDATION Flag para indicar se a orientação é válida
  //PARA ROS2 
  , linear_velocity_(Eigen::Vector3d::Zero()) // Velocidade linear inicial (0,0,0)
  , angular_velocity_(Eigen::Vector3d::Zero())  // Velocidade angular inicial (0,0,0)

  // , have_odom_(false)//FLAG ODOM START
  // , have_bias_(false) 
{
  // nh_private.param("max_imu_queue_length", max_imu_queue_length_, 1000);
 // EQUIVALENTE PARA ROS2: 
  this->declare_parameter<int>("max_imu_queue_length", 1000);
  max_imu_queue_length_ = this->get_parameter("max_imu_queue_length").as_int();
  transform_.setIdentity(); 

  constexpr size_t kROSQueueLength = 100;
  imu_sub_ = this->create_subscription<unitree_go::msg::LowState>("lowstate", kROSQueueLength, std::bind(&OdomPredictor::lowstateCallback, this, std::placeholders::_1));
 // Initialize broadcaster PARA ROS2:
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("imu_odometry", kROSQueueLength);
  transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("imu_transform", kROSQueueLength);

  
  // transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
  //     "imu_transform", kROSQueueLength);//ROS1 PUBLICA TF EM IMU_TRANSFORM

//mudança local até linha 56 - PARA ROS2
 
  geometry_msgs::msg::Pose pose;// Cria uma pose zerada
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0.2;  // Altura inicial sugerida (0.2m)
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1.0;  // Sem rotação
  
  // pose.position = pos;

    // CONVERTER PARA ROS2
      transform_.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
      transform_.linear() = Eigen::Quaterniond(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z
    ).toRotationMatrix();

    

  // tf::poseMsgToKindr(pose, &transform_);//ROS1 CONVERTE POSE PARA KINDR

  // /*
  // c =                 {1 0 0 0 0 0
  //                     0 1 0 0 0 0 
  //                     0 0 1 0 0 0
  //                     0 0 0 1 0 0 
  //                     0 0 0 0 1 0 
  //                     0 0 0 0 0 1};
  // */
  // linear_velocity_ = {0, 0, 0};//INIT 
  // angular_velocity_ = {0, 0, 0};
  // /*
  // twist_covariance_ = {1 0 0 0 0 0
  //                     0 1 0 0 0 0 
  //                     0 0 1 0 0 0
  //                     0 0 0 1 0 0 
  //                     0 0 0 0 1 0 
  //                     0 0 0 0 0 1};
  // */
  frame_id_ = "odom";
  child_frame_id_ = "base_link";
}


void OdomPredictor::lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
//TYPE OF MESSAGES TO RECEIVE 
  // if (msg->orientation_covariance[0] == -1.0) {
  //   have_orientation_ = false;
  // }
  // else {
  //   tf::quaternionMsgToKindr(msg->orientation, &orientation_);
  //   transform_.getRotation() = orientation_;
  // }


  // if (msg->tick < states_queue_.back().tick) {
  //   RCLCPP_ERROR_STREAM(rclcpp::get_logger("imu_to_odom"),
  //                       "Latest LowState message occured at time: "
  //                         << msg->tick
  //                         << ". This is before the previously received LowState "
  //                             "message that ouccured at: "
  //                         << states_queue_.back().tick
  //                         << ". The current states queue will be reset.");
  //   states_queue_.clear();
  // }

  states_queue_.push_back(*msg);//KEEP DATA IN THE QUEUE

  try {
    integrateIMUData(msg);
  } catch (std::exception& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("imu_to_odom"),
        "IMU INTEGRATION FAILED, RESETING EVERYTHING: " << e.what());
    // have_bias_ = false;
    // have_odom_ = false;
    states_queue_.clear();
    return;
  }

  publishOdometry();
  publishTransform();
  ++seq_;
}

  void OdomPredictor::integrateIMUData(const unitree_go::msg::LowState::SharedPtr msg) {
  
    rclcpp::Time current_time = this->now(); 

  // if (!has_imu_meas) {
  //   estimate_timestamp_ = msg->tick;//ARMAZENA TIMESTAMP PRIMEIRA MEAS
  //   has_imu_meas = true;
  //   return;
  // }

    if (!has_imu_meas) {
    estimate_timestamp_ = current_time;//ARMAZENA TIMESTAMP PRIMEIRA MEAS
    has_imu_meas = true;
    return;
  }

  const double delta_time =( current_time - estimate_timestamp_).seconds();// TEMPO DESDE A ULTIMA MEDIÇÃO
  estimate_timestamp_ = current_time;

  //DADOS IMU
  const Eigen::Vector3d kGravity(0.0, 0.0, -9.81);

  Eigen::Vector3d imu_linear_acceleration, imu_angular_velocity;
  imu_linear_acceleration << msg->imu_state.accelerometer[0], msg->imu_state.accelerometer[1], msg->imu_state.accelerometer[2];
  imu_angular_velocity << msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1], msg->imu_state.gyroscope[2];


  //MÉTODO TRAPEZOIDAL DE INTEGRAR A VELOCIDADE ANGULAR:
  const Eigen::Vector3d final_angular_velocity = (imu_angular_velocity - imu_angular_velocity_bias_);
  const Eigen::Vector3d delta_angle = delta_time * (final_angular_velocity + angular_velocity_) / 2.0;
  angular_velocity_ = final_angular_velocity;

  // apply half of the rotation delta - ROTAÇÃO INCREMENTAL -INTEGRAÇÃO MAIS PRECISA
  double angle = delta_angle.norm()/2; // Calculate the angle (magnitude of the vector)
  Eigen::Vector3d axis = delta_angle.normalized(); // Normalize the vector to get the axis
  Eigen::AngleAxisd angle_axis(angle, axis);
  const Eigen::Quaterniond half_delta_rotation(angle_axis);//CONVERTS TO QUARTENION

  
  if (!have_orientation_) {
  //   transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
  //PARA ROS2:
    Eigen::Quaterniond current_rotation(transform_.linear());
    current_rotation = current_rotation * half_delta_rotation;
    transform_.linear() = current_rotation.toRotationMatrix();

  }

  //find changes in linear velocity and position- INTEGRAÇÃO LINEAR
  const Eigen::Vector3d delta_linear_velocity = 
      delta_time * (imu_linear_acceleration 
                     - transform_.linear().transpose() * kGravity  //PARA ROS2            
                   - imu_linear_acceleration_bias_);


  //ATUALIZA A VEL MEDIA USANDO DELTA TIME
  // transform_.getPosition() =
  //     transform_.getPosition() +
  //     transform_.getRotation().rotate(
  //         delta_time * (linear_velocity_ + delta_linear_velocity / 2.0));// CORRIGE A DIREÇÃODA VEL CONFORME A ATUAL
  // linear_velocity_ += delta_linear_velocity;//UPTES

  // if (!have_orientation_) {
  // // apply the other half of the rotation delta
  //   transform_.getRotation() = transform_.getRotation() * half_

  //PARA ROS2
    transform_.translation() += 
      transform_.linear() * 
      (delta_time * (linear_velocity_ + delta_linear_velocity / 2.0));
  
  linear_velocity_ += delta_linear_velocity;

  if (!have_orientation_) {
    // Apply the other half of the rotation delta
    Eigen::Quaterniond current_rotation(transform_.linear());
    current_rotation = current_rotation * half_delta_rotation;
    transform_.linear() = current_rotation.toRotationMatrix();
  }
  

  // estimate_timestamp_ = msg->tick;
}


void OdomPredictor::publishOdometry() {
//PARA ROS2 
  nav_msgs::msg::Odometry msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.child_frame_id = child_frame_id_;

  // Position and orientation
  msg.pose.pose.position.x = transform_.translation().x();
  msg.pose.pose.position.y = transform_.translation().y();
  msg.pose.pose.position.z = transform_.translation().z();
  
  Eigen::Quaterniond q(transform_.linear());
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // Velocity
  msg.twist.twist.linear.x = linear_velocity_.x();
  msg.twist.twist.linear.y = linear_velocity_.y();
  msg.twist.twist.linear.z = linear_velocity_.z();
  
  msg.twist.twist.angular.x = angular_velocity_.x();
  msg.twist.twist.angular.y = angular_velocity_.y();
  msg.twist.twist.angular.z = angular_velocity_.z();

  odom_pub_->publish(msg);


  // nav_msgs::Odometry msg;

  // msg.header.frame_id = frame_id_;
  // msg.header.seq = seq_;
  // msg.header.stamp = estimate_timestamp_;
  // msg.child_frame_id = child_frame_id_;

  // tf::poseKindrToMsg(transform_, &msg.pose.pose);
 

  // //msg.pose.covariance = pose_covariance_;

  // tf::vectorKindrToMsg(linear_velocity_, &msg.twist.twist.linear);
  // tf::vectorKindrToMsg(angular_velocity_, &msg.twist.twist.angular);
  // //msg.twist.covariance = twist_covariance_;

  // odom_pub_.publish(msg);//CONVERTE DE KINDR PARA ROS1
}

void OdomPredictor::publishTransform() {
//PARA ROS2 
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.child_frame_id = child_frame_id_;

  // Translation
  msg.transform.translation.x = transform_.translation().x();
  msg.transform.translation.y = transform_.translation().y();
  msg.transform.translation.z = transform_.translation().z();

  // Rotation
  Eigen::Quaterniond q(transform_.linear());
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();

  transform_pub_->publish(msg);
}

//   geometry_msgs::TransformStamped msg;

//   msg.header.frame_id = frame_id_;
//   msg.header.seq = seq_;
//   msg.header.stamp = estimate_timestamp_;
//   msg.child_frame_id = child_frame_id_;

//   tf::transformKindrToMsg(transform_, &msg.transform);
//   transform_pub_.publish(msg);
//   br_.sendTransform(msg);//ESSE BLOCO CONVERTE E PUBLICA EM ROS1
// }

