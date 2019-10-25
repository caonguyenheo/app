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

if (ENABLE_ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS)
    find_package(PkgConfig REQUIRED)
    find_package(roscpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(novatel_gps_msgs REQUIRED)
    find_package(catkin REQUIRED roscpp sensor_msgs geometry_msgs novatel_gps_msgs)
    include_directories(include ${catkin_INCLUDE_DIRS})
#    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif ()

if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
endif()

find_package(Qt5Core REQUIRED)
find_package(Qt5GStreamer REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Network REQUIRED)

include_directories(
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

target_link_libraries(${PROJECT_NAME} Qt5::Core ${catkin_LIBRARIES} ${QTGSTREAMER_UTILS_LIBRARIES} Qt5::Gui Qt5::Network)
target_link_libraries(${PROJECT_NAME} spdlog::spdlog)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

