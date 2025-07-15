cmake_minimum_required(VERSION 3.8)
project(imu_to_odom)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rosidl_runtime_c REQUIRED)
# find_package(rosidl REQUIRED)
# find_package(std_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  SHARED
    src/imu_to_odom.cpp
)

# target_link_libraries(${PROJECT_NAME}
#   INTERFACE
#   rosidl_runtime_c::rosidl_runtime_c
# )

ament_target_dependencies(${PROJECT_NAME}
  std_msgs
)

############
# BINARIES #
############
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


ament_export_include_directories(
  include
)

ament_export_libraries (
  go2_jointcontroller
)


# ✅ Register Package
ament_package()
