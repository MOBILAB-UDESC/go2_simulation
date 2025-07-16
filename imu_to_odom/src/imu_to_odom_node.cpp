#include "imu_to_odom/imu_to_odom.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPredictor>());
  rclcpp::shutdown();
  return 0;
}
