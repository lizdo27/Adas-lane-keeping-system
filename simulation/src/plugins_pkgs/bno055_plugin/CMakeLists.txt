cmake_minimum_required(VERSION 3.5)
project(bno055_plugin)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(bno055_plugin SHARED src/bno055_plugin.cpp)
target_include_directories(bno055_plugin PUBLIC include)
target_link_libraries(bno055_plugin 
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS bno055_plugin
  DESTINATION lib)

ament_package()

