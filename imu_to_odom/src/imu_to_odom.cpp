#include "imu_to_odom/imu_to_odom.h"

OdomPredictor::OdomPredictor()
  : Node("OdomPredictor")
  , seq_(0)
  , has_imu_meas(false)
  , imu_linear_acceleration_bias_(0, 0, 0)
  , imu_angular_velocity_bias_(0, 0, 0)
  , have_orientation_(true)
  , transform_(Eigen::Affine3d::Identity())
  // , have_odom_(false)
  // , have_bias_(false) 
{
  // nh_private.param("max_imu_queue_length", max_imu_queue_length_, 1000);

  constexpr size_t kROSQueueLength = 100;
  imu_sub_ = this->create_subscription<unitree_go::msg::LowState>("lowstate", kROSQueueLength, std::bind(&OdomPredictor::lowstateCallback, this, std::placeholders::_1));
 
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("imu_odometry", kROSQueueLength);
  
  // transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
  //     "imu_transform", kROSQueueLength);


  // geometry_msgs::Point pos;
  // geometry_msgs::Pose pose;

  // pos.x = 0; 
  // pos.y = 0; 
  // pos.z = 1.0;
  
  // geometry_msgs::Quaternion quat;
  // quat.x = 0;
  // quat.y = 0;
  // quat.z = 0;
  // quat.w = 1.0;
  // pose.orientation = quat;
  
  // pose.position = pos;

  // tf::poseMsgToKindr(pose, &transform_);

  // /*
  // c =                 {1 0 0 0 0 0
  //                     0 1 0 0 0 0 
  //                     0 0 1 0 0 0
  //                     0 0 0 1 0 0 
  //                     0 0 0 0 1 0 
  //                     0 0 0 0 0 1};
  // */
  // linear_velocity_ = {0, 0, 0};
  // angular_velocity_ = {0, 0, 0};
  // /*
  // twist_covariance_ = {1 0 0 0 0 0
  //                     0 1 0 0 0 0 
  //                     0 0 1 0 0 0
  //                     0 0 0 1 0 0 
  //                     0 0 0 0 1 0 
  //                     0 0 0 0 0 1};
  // */
  // frame_id_ = "world";
  // child_frame_id_ = "odom";
}


void OdomPredictor::lowstateCallback(const unitree_go::msg::LowState::SharedPtr msg) {
  // if (msg->orientation_covariance[0] == -1.0) {
  //   have_orientation_ = false;
  // }
  // else {
  //   tf::quaternionMsgToKindr(msg->orientation, &orientation_);
  //   transform_.getRotation() = orientation_;
  // }


  if (msg->tick < states_queue_.back().tick) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("imu_to_odom"),
                        "Latest LowState message occured at time: "
                          << msg->tick
                          << ". This is before the previously received LowState "
                              "message that ouccured at: "
                          << states_queue_.back().tick
                          << ". The current states queue will be reset.");
    states_queue_.clear();
  }

  states_queue_.push_back(*msg);

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
  // publishTF();
  ++seq_;
}

void OdomPredictor::integrateIMUData(const unitree_go::msg::LowState::SharedPtr msg) {
  if (!has_imu_meas) {
    estimate_timestamp_ = msg->tick;
    has_imu_meas = true;
    return;
  }

  const double delta_time = msg->tick - estimate_timestamp_;

  const Eigen::Vector3d kGravity(0.0, 0.0, -9.81);

  Eigen::Vector3d imu_linear_acceleration, imu_angular_velocity;
  imu_linear_acceleration << msg->imu_state.accelerometer[0], msg->imu_state.accelerometer[1], msg->imu_state.accelerometer[2];
  imu_angular_velocity << msg->imu_state.gyroscope[0], msg->imu_state.gyroscope[1], msg->imu_state.gyroscope[2];

  const Eigen::Vector3d final_angular_velocity = (imu_angular_velocity - imu_angular_velocity_bias_);
  const Eigen::Vector3d delta_angle = delta_time * (final_angular_velocity + angular_velocity_) / 2.0;
  angular_velocity_ = final_angular_velocity;

  // apply half of the rotation delta
  double angle = delta_angle.norm(); // Calculate the angle (magnitude of the vector)
  Eigen::Vector3d axis = delta_angle.normalized(); // Normalize the vector to get the axis
  Eigen::AngleAxisd angle_axis(angle/2, axis);

  const Eigen::Quaterniond half_delta_rotation(angle_axis);

  if (!have_orientation_) {
    // transform_.rotation() = transform_.rotation() * half_delta_rotation;

    rotation_ = rotation_ * half_delta_rotation;
  }

  // find changes in linear velocity and position
  const Eigen::Vector3d delta_linear_velocity = 
      delta_time * (imu_linear_acceleration +
                    // rotation_.inverse().rotate(kGravity) -
                    imu_linear_acceleration_bias_);
  // transform_.getPosition() =
  //     transform_.getPosition() +
  //     transform_.getRotation().rotate(
  //         delta_time * (linear_velocity_ + delta_linear_velocity / 2.0));
  // linear_velocity_ += delta_linear_velocity;

  transform_.translation() += transform_.rotation() * (delta_time * (linear_velocity_ + delta_linear_velocity / 2.0));
    linear_velocity_ += delta_linear_velocity;

  if (!have_orientation_) {
  // // apply the other half of the rotation delta
  //   transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
  }

  estimate_timestamp_ = msg->tick;
}

void OdomPredictor::publishOdometry() {
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

  // odom_pub_.publish(msg);
}

// void OdomPredictor::publishTF() {
//   geometry_msgs::TransformStamped msg;

//   msg.header.frame_id = frame_id_;
//   msg.header.seq = seq_;
//   msg.header.stamp = estimate_timestamp_;
//   msg.child_frame_id = child_frame_id_;

//   tf::transformKindrToMsg(transform_, &msg.transform);

//   transform_pub_.publish(msg);
//   br_.sendTransform(msg);
// }

