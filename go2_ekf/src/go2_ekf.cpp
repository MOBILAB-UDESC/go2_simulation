#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Go2EKFNode : public rclcpp::Node
{
public:
  Go2EKFNode()
  : Node("go2_ekf_node")
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    imu_sub_ = this->create_subscription<unitree_go::msg::IMUState>(
      "/imu", 10,
      std::bind(&Go2EKFNode::imu_callback, this, std::placeholders::_1));
  }

private:
  void imu_callback(const unitree_go::msg::IMUState::SharedPtr msg)
  {
    sensor_msgs::msg::Imu imu_msg;

    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";  // You can change this if needed

    // Orientation
    imu_msg.orientation.x = msg->quaternion[0];
    imu_msg.orientation.y = msg->quaternion[1];
    imu_msg.orientation.z = msg->quaternion[2];
    imu_msg.orientation.w = msg->quaternion[3];

    // Angular velocity
    imu_msg.angular_velocity.x = msg->gyroscope[0];
    imu_msg.angular_velocity.y = msg->gyroscope[1];
    imu_msg.angular_velocity.z = msg->gyroscope[2];

    // Linear acceleration
    imu_msg.linear_acceleration.x = msg->accelerometer[0];
    imu_msg.linear_acceleration.y = msg->accelerometer[1];
    imu_msg.linear_acceleration.z = msg->accelerometer[2];

    imu_pub_->publish(imu_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<unitree_go::msg::IMUState>::SharedPtr imu_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2EKFNode>());
  rclcpp::shutdown();
  return 0;
}
