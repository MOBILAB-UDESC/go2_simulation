#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Go2FixedCovariance : public rclcpp::Node
{
public:
  Go2FixedCovariance() : Node("go2_fixed_covariance")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10,
      std::bind(&Go2FixedCovariance::imuCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_fixed", 10);

    RCLCPP_INFO(this->get_logger(), "go2_fixed_covariance node started");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto patched = *msg;

    // Inject fake covariances
    patched.orientation_covariance[0] = 0.001;
    patched.orientation_covariance[1] = 0.001;
    patched.orientation_covariance[2] = 0.001;
    patched.orientation_covariance[3] = 0.001;
    patched.orientation_covariance[4] = 0.001;
    patched.orientation_covariance[5] = 0.001;
    patched.orientation_covariance[6] = 0.001;
    patched.orientation_covariance[7] = 0.001;
    patched.orientation_covariance[8] = 0.001;

    patched.angular_velocity_covariance[0] = 0.001;
    patched.angular_velocity_covariance[1] = 0.001;
    patched.angular_velocity_covariance[2] = 0.001;
    patched.angular_velocity_covariance[3] = 0.001;
    patched.angular_velocity_covariance[4] = 0.001;
    patched.angular_velocity_covariance[5] = 0.001;
    patched.angular_velocity_covariance[6] = 0.001;
    patched.angular_velocity_covariance[7] = 0.001;
    patched.angular_velocity_covariance[8] = 0.001;

    patched.linear_acceleration_covariance[0] = 0.01;
    patched.linear_acceleration_covariance[1] = 0.01;
    patched.linear_acceleration_covariance[2] = 0.01;
    patched.linear_acceleration_covariance[3] = 0.01;
    patched.linear_acceleration_covariance[4] = 0.01;
    patched.linear_acceleration_covariance[5] = 0.01;
    patched.linear_acceleration_covariance[6] = 0.01;
    patched.linear_acceleration_covariance[7] = 0.01;
    patched.linear_acceleration_covariance[8] = 0.01;

    pub_->publish(patched);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2FixedCovariance>());
  rclcpp::shutdown();
  return 0;
}
