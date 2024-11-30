cmake_minimum_required(VERSION 3.8)
project(spiderproj_nrf24)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(rf24_node src/rf24_node.cpp)
target_link_libraries(rf24_node rf24)
ament_target_dependencies(rf24_node rclcpp sensor_msgs)

install(TARGETS
  rf24_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
