cmake_minimum_required(VERSION 3.8)
project(spiderproj_servo2040com)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spiderproj_msgs REQUIRED)

add_executable(servo2040_node src/servo2040_node.cpp)
target_link_libraries(servo2040_node)
ament_target_dependencies(servo2040_node rclcpp spiderproj_msgs)

install(TARGETS
  servo2040_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
