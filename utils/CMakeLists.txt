cmake_minimum_required(VERSION 3.15)
project(utils)

add_compile_options("-std=c++2a")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
	SHARED
		src/pose.cpp
		src/screw.cpp
)

# target_link_libraries(${PROJECT_NAME}
# 	PUBLIC
# 		# robotlib
# 		# yaml-cpp
# 		# dls_math
# 		# dls_time
# )

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		include
		${EIGEN3_INCLUDE_DIR}
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(
  include
)

ament_export_libraries (
  ${PROJECT_NAME}
)

ament_package()