cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pedestrian_object_plugin)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(pedestrian_object_plugin SHARED
 src/geometric_point.cpp
 src/pedestrian_object_plugin.cpp 
 include/geometric_point.h
 include/pedestrian_object_plugin.h
)

target_include_directories(pedestrian_object_plugin PUBLIC include)
target_link_libraries(pedestrian_object_plugin 
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

install(TARGETS pedestrian_object_plugin
  DESTINATION lib)

ament_package()
