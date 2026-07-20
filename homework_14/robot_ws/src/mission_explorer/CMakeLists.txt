cmake_minimum_required(VERSION 3.20)
project(mission_explorer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(mission_explorer_node src/mission_explorer_node.cpp)
target_include_directories(mission_explorer_node PRIVATE include)

ament_auto_package()
