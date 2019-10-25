cmake_minimum_required(VERSION 3.5)
project (controlsystem)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-attributes)
endif()

add_subdirectory( katricontrols )

IF(UNIX)
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:\$ORIGIN")
ENDIF()

# declaring files in your project
set ( SOURCES
    main.cpp
    mainwindow.cpp
    katrilogger/appConfig.cpp
    katrilogger/command.cpp
    katrilogger/nodeWorker.cpp
    katrilogger/runtime.cpp
    pattern/Mediator.cpp
    pattern/Message.cpp
    logger/QLogger.cpp
    katrilogger/socketthread.cpp
)

set ( MOC_HEADERS
    mainwindow.h
    katrilogger/appConfig.h
    katrilogger/command.h
    katrilogger/nodeWorker.h
    katrilogger/runtime.h
    pattern/Mediator.h
    pattern/Message.h
    logger/QLogger.h
    katrilogger/socketthread.h
)

set ( MOC_UI
    )

# Next lines needed for building all Qt projects
FIND_PACKAGE(Qt5Core)
FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5Network)

# ROS2 dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(katri_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)

add_definitions( ${QT_DEFINITIONS} )
include_directories( ${CMAKE_BINARY_DIR} )

include_directories(
    ${CMAKE_SOURCE_DIR}/katricontrols
    ${CMAKE_BINARY_DIR}/katricontrols
    ${rclcpp_INCLUDE_DIRS}
)

# using Qt meta-system (precompiler)
QT5_WRAP_CPP( MOC_SRCS ${MOC_HEADERS} )
qt5_wrap_ui( UI_HEADERS ${MOC_UI} )
#qt5_add_resources(SOURCES ${QRCS})

# compile
add_executable(${PROJECT_NAME} ${SOURCES} ${MOC_SRCS} ${UI_HEADERS})

# ROS libs
#ament_target_dependencies(${PROJECT_NAME} rclcpp katri_msgs sensor_msgs)

# build it (link libraries)
target_link_libraries(${PROJECT_NAME} katricontrols Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Network)
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

#ament_package()
