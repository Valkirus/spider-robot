cmake_minimum_required(VERSION 3.8)
project(spiderproj_keyboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spiderproj_msgs REQUIRED)

add_executable(keyboard_node src/keyboard_node.cpp)
ament_target_dependencies(keyboard_node rclcpp spiderproj_msgs)

install(TARGETS
  keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
