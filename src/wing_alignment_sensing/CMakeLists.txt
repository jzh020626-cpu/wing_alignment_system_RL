cmake_minimum_required(VERSION 3.8)
project(wing_alignment_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)

add_executable(qr_delta_publisher src/qr_delta_publisher.cpp)
target_include_directories(qr_delta_publisher PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(qr_delta_publisher rclcpp geometry_msgs std_srvs)

add_executable(force_monitor src/force_monitor.cpp)
target_include_directories(force_monitor PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(force_monitor rclcpp std_msgs std_srvs)

install(TARGETS
  qr_delta_publisher
  force_monitor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
