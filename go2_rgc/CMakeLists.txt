cmake_minimum_required(VERSION 3.8)
project(go2_rgc)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wno-pedantic)
# endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(unitree_go REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(osqp REQUIRED)
# find_package(osqp-cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(
    osqp_cpp
    GIT_REPOSITORY https://github.com/google/osqp-cpp.git
    GIT_TAG master # or a specific release tag
)
FetchContent_MakeAvailable(osqp_cpp)


# Include directories
# include_directories(
#   include
#   ${pinocchio_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
# )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Build main library (go2_rgc.cpp)
add_library(${PROJECT_NAME}
  SHARED
  src/go2_rgc.cpp
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  osqp::osqp
  osqp-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  Eigen3
  unitree_go
  controller_interface
  pluginlib
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(controller_interface go2_rgc.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include DESTINATION include/go2_rgc)
install(FILES ${DDS_LIBRARIES} DESTINATION lib)


ament_export_include_directories(
  include
)

ament_export_libraries (
  go2_rgc
  unitree_go
)
  
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  go2_interfaces
)

ament_package()

# target_include_directories(${PROJECT_NAME} PUBLIC include)

# # Build node executable
# add_executable(rgc_node src/rgc_node.cpp)
# ament_target_dependencies(rgc_node
#   rclcpp
#   sensor_msgs
#   geometry_msgs
# )
# target_link_libraries(${PROJECT_NAME}  
#  pinocchio::pinocchio
#  )

# # Install targets
# install(TARGETS
#   rgc_node
#   ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Testing (optional)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
