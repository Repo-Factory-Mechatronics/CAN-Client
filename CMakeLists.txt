cmake_minimum_required(VERSION 3.5)
project(can_client)

set(CMAKE_CXX_STANDARD 20)

find_package(rclcpp REQUIRED)
find_package(scion_types REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(ROBOT_TYPES robot_types)

include_directories(include)

# Add your package's source files
add_library(
    can_client
    SHARED 
    src/can_client.cpp
)

target_link_libraries(can_client
  ${ROBOT_TYPES}
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

file(GLOB HEADERS include/can_client.hpp)

install(FILES ${HEADERS} DESTINATION include/robot_libraries)

ament_target_dependencies(can_client scion_types rclcpp std_srvs std_msgs sensor_msgs)
