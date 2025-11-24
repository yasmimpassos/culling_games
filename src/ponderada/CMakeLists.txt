cmake_minimum_required(VERSION 3.8)
project(ponderada)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

add_executable(parte1
  src/parte1.cpp
)

ament_target_dependencies(parte1 rclcpp cg_interfaces)

install(TARGETS
  parte1
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(parte2
  src/parte2.cpp
)

ament_target_dependencies(parte2 rclcpp cg_interfaces)

install(TARGETS
  parte2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
