cmake_minimum_required(VERSION 3.8)
project(imu_to_odom)

set(CMAKE_CXX_STANDARD 17)

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(rosidl_runtime_c REQUIRED)
# find_package(rosidl REQUIRED)
# find_package(std_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
  SHARED
    src/imu_to_odom.cpp
)

# target_link_libraries(${PROJECT_NAME}
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   unitree_go
# )

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  nav_msgs
  unitree_go
)

add_executable(imu_to_odom_node
  src/imu_to_odom_node.cpp
)

target_link_libraries(imu_to_odom_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} imu_to_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/
)

ament_package()
