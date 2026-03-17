cmake_minimum_required(VERSION 3.15)
project(go2_trunkcontroller)

add_compile_options("-std=c++2a")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(utils REQUIRED)
find_package(pinocchio REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(pinocchio REQUIRED)
# find_package(unitree_go REQUIRED)
# find_package(controller_interface REQUIRED)
# find_package(ament_index_cpp REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(osqp REQUIRED)
# # find_package(osqp-cpp REQUIRED)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Build main library (go2_rgc.cpp)
add_library(${PROJECT_NAME}
  SHARED
    src/trunk_controller.cpp
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
# # #   osqp::osqp
# # #   osqp-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  utils
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ DESTINATION config/)

ament_export_include_directories(
  include
)

ament_export_libraries (
  go2_trunkcontroller
  # unitree_go
)
  
# ament_export_dependencies(
#   controller_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   go2_interfaces
# )

add_executable(go2_trunk_motion_generator  
  src/trunk_motion_generator.cpp
)

target_include_directories(go2_trunk_motion_generator
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIR}
)


# ament_target_dependencies(go2_trunk_motion_generator
#   rclcpp
#   unitree_go
# )

ament_target_dependencies(go2_trunk_motion_generator
  Eigen3
  utils
)

install(TARGETS
  go2_trunk_motion_generator
  DESTINATION lib/go2_trunk_motion_generator
)

ament_package()