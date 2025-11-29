cmake_minimum_required(VERSION 3.8)
project(ponderada)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include_directories(include)

add_executable(part1
  src/part1.cpp
  src/get_map_client.cpp
  src/map_graph.cpp
  src/map_printer.cpp
)

ament_target_dependencies(part1
  rclcpp
  cg_interfaces
)

install(TARGETS part1
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
