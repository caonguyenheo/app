cmake_minimum_required(VERSION 3.5)
project(katri-control-console)

set(CMAKE_AUTOMOC ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Qt5Core REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(katri_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
 ${rclcpp_INCLUDE_DIRS}
 )

file(GLOB SOURCE_FILES
 "src/*.cpp"
 )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp katri_msgs sensor_msgs)

target_link_libraries(${PROJECT_NAME} Qt5::Core)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
