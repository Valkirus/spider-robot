cmake_minimum_required(VERSION 3.8)
project(spiderproj_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(spiderproj_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3)

add_executable(spiderproj_kinematics src/kinematics.cpp)
ament_target_dependencies(spiderproj_kinematics rclcpp std_msgs orocos_kdl spiderproj_msgs sensor_msgs)

install(TARGETS 
  spiderproj_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()