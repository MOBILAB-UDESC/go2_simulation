cmake_minimum_required(VERSION 3.8)
project(go2_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)  # your custom message package
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

add_executable(go2_ekf src/go2_ekf.cpp)

ament_target_dependencies(go2_ekf
  rclcpp
  sensor_msgs
  std_msgs
  unitree_go
)

install(TARGETS
  go2_ekf
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go2_fixed_covariance src/go2_fixed_covariance.cpp)

ament_target_dependencies(go2_fixed_covariance 
  rclcpp 
  sensor_msgs
)

install(TARGETS 
  go2_fixed_covariance 
  DESTINATION lib/${PROJECT_NAME}
)
