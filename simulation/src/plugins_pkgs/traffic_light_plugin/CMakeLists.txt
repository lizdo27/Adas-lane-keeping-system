cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(traffic_light_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(traffic_light_plugin SHARED src/traffic_light_plugin.cpp include/traffic_light_plugin.hpp)
target_include_directories(traffic_light_plugin PUBLIC include)
target_link_libraries(traffic_light_plugin 
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)
ament_target_dependencies(traffic_light_plugin rclcpp std_msgs)

install(TARGETS traffic_light_plugin
  DESTINATION lib)

ament_package()
