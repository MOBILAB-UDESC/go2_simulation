#include "imu_to_odom/imu_to_odom.h"

OdomPredictor::OdomPredictor()
  : Node("OdomPredictor")
  , seq_(0)
  , has_imu_meas(false)
  , imu_linear_acceleration_bias_(0, 0, 0)
  , imu_angular_velocity_bias_(0, 0, 0)
  , have_orientation_(true)
  // , have_odom_(false)
  // , have_bias_(false) 
{

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
  linear_velocity_ = {0, 0, 0};
  angular_velocity_ = {0, 0, 0};
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

// void OdomPredictor::imuBiasCallback(const sensor_msgs::ImuConstPtr& msg) {
//   tf::vectorMsgToKindr(msg->linear_acceleration,
//                        &imu_linear_acceleration_bias_);
//   tf::vectorMsgToKindr(msg->angular_velocity, &imu_angular_velocity_bias_);

//   have_bias_ = true;
// }

void OdomPredictor::integrateIMUData(const unitree_go::msg::LowState::SharedPtr msg) {
  if (!has_imu_meas) {
    estimate_timestamp_ = msg->tick;
    has_imu_meas = true;
    return;
  }

  const double delta_time = msg->tick - estimate_timestamp_;

  const Vector3 kGravity(0.0, 0.0, -9.81);

  Vector3 imu_linear_acceleration, imu_angular_velocity;
  vectorMsgToKindr(msg->imu_state.accelerometer, &imu_linear_acceleration);
  vectorMsgToKindr(msg->imu_state.gyroscope, &imu_angular_velocity);

  const Vector3 final_angular_velocity =
      (imu_angular_velocity - imu_angular_velocity_bias_);
  const Vector3 delta_angle =
      delta_time * (final_angular_velocity + angular_velocity_) / 2.0;
  angular_velocity_ = final_angular_velocity;

  // apply half of the rotation delta
  const Rotation half_delta_rotation = Rotation::exp(delta_angle / 2.0);

  if (!have_orientation_) {
    transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
  }

  // find changes in linear velocity and position
  const Vector3 delta_linear_velocity =
      delta_time * (imu_linear_acceleration +
                    transform_.getRotation().inverse().rotate(kGravity) -
                    imu_linear_acceleration_bias_);
  transform_.getPosition() =
      transform_.getPosition() +
      transform_.getRotation().rotate(
          delta_time * (linear_velocity_ + delta_linear_velocity / 2.0));
  linear_velocity_ += delta_linear_velocity;

  if (!have_orientation_) {
  // apply the other half of the rotation delta
    transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
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

// template <typename Scalar>
void OdomPredictor::vectorMsgToKindr(const std::array<float, 3>& msg, Eigen::Vector3d* kindr) {
  if (kindr == nullptr)
    return;
  Eigen::Vector3d kindr_double;
  kindr_double(0) = msg[0]; 
  kindr_double(1) = msg[1]; 
  kindr_double(2) = msg[2]; 
  *kindr = kindr_double;
}

