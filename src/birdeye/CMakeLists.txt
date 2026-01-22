cmake_minimum_required(VERSION 3.8)
project(birdeye)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(birdeye src/birdeye.cpp)
ament_target_dependencies(birdeye rclcpp sensor_msgs cv_bridge)
target_link_libraries(birdeye ${OpenCV_LIBS})

install(TARGETS birdeye DESTINATION lib/${PROJECT_NAME})
ament_package()
