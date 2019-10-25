cmake_minimum_required(VERSION 3.5)

project(katri_logger VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${QTGSTREAMER_FLAGS}")

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Set 1 to enable ROS version, otherwise disable it, only ROS 2
message(${CMAKE_SYSTEM_PROCESSOR})
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)|(x86_64)|(X86_64)")
    set(ENABLE_ROS_VERSION 1)
else ()
    set(ENABLE_ROS_VERSION 0)
endif ()

configure_file(
    ${PROJECT_SOURCE_DIR}/src/config.h.in
    ${PROJECT_SOURCE_DIR}/src/config.h
    )

# ROS1
if (ENABLE_ROS_VERSION EQUAL 1)
find_package(rmw_implementation_cmake REQUIRED)

# find ROS 1 packages
set(cmake_extras_files cmake/find_ros1_package.cmake cmake/find_ros1_interface_packages.cmake)
include(cmake/find_ros1_package.cmake)

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(WARNING "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  message(WARNING "Failed to find ROS 1 roscpp, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package(
    CONFIG_EXTRAS ${cmake_extras_files}
  )
  return()
endif()

find_ros1_package(sensor_msgs)
if(NOT ros1_sensor_msgs_FOUND)
    message("not found sensor_msgs")
endif()

find_ros1_package(novatel_gps_msgs)
if(NOT ros1_novatel_gps_msgs_FOUND)
    message("not found novatel_gps_msgs")
endif()

find_ros1_package(geometry_msgs)
if(NOT ros1_geometry_msgs_FOUND)
    message("not found geometry_msgs")
endif()
# End ROS1
endif()

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

find_package(Qt5Core REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(katri_msgs REQUIRED)
find_package(Qt5GStreamer REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5Network REQUIRED)

include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${QTGSTREAMER_INCLUDES}
 )

file(GLOB SOURCE_FILES
    "src/*.cpp"
    "src/camera/*.cpp"
    "src/can/*.cpp"
    "src/gps/*.cpp"
    "src/lidar/*.cpp"
 )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

ament_target_dependencies(${PROJECT_NAME} rclcpp katri_msgs sensor_msgs)

if (ENABLE_ROS_VERSION EQUAL 1)
    ament_target_dependencies(${PROJECT_NAME} ros1_roscpp ros1_sensor_msgs ros1_katri_msgs ros1_geometry_msgs ros1_novatel_gps_msgs)
endif()

target_link_libraries(${PROJECT_NAME} Qt5::Core ${QTGSTREAMER_UTILS_LIBRARIES} Qt5::Gui Qt5::Network)
target_link_libraries(${PROJECT_NAME} spdlog::spdlog)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
