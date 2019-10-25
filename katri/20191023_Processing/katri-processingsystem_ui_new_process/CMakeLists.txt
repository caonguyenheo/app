cmake_minimum_required(VERSION 3.7)
project(Processor)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

add_subdirectory(katricontrols)

# declaring files in your project
set(application_SOURCES
        main.cpp
        mainwindow.cpp
        subwindow_export.cpp
        src/extractor/kittiextractor.cpp
        src/extractor/imageextraction.cpp
        katricontrols/common.cpp
        src/common/appConfig.cpp
#        src/common/qarchive.cpp
        src/mediator/IMediator.cpp
        src/broadcast/CVideoPlayer.cpp
        src/broadcast/CLidarSimulator.cpp
        src/broadcast/CGpsSimulator.cpp
        src/broadcast/CCanSimulator.cpp
        src/broadcast/CMediaControl.cpp
        src/broadcast/SchedulerThread.cpp
        )

set(application_HEADERS
        mainwindow.h
        subwindow_export.h
        src/extractor/kittiextractor.h
        src/extractor/imageextraction.h
        katricontrols/common.h
        src/common/appConfig.h
#        src/common/qarchive.h
        src/mediator/IMediator.h
        src/mediator/IColleague.h
        src/broadcast/CVideoPlayer.h
        src/broadcast/CLidarSimulator.h
        src/broadcast/CGpsSimulator.h
        src/broadcast/CCanSimulator.h
        src/broadcast/CMediaControl.h
        src/broadcast/SchedulerThread.h
        )

set(application_UIS
        mainwindow.ui
        )

# Next lines needed for building all Qt projects
FIND_PACKAGE(Qt5Core)
FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5QuickWidgets REQUIRED)

add_definitions(${QT_DEFINITIONS})
include_directories(${CMAKE_BINARY_DIR})

include_directories(
        ${CMAKE_SOURCE_DIR}/katricontrols
        ${CMAKE_BINARY_DIR}/katricontrols
)

# using Qt meta-system (precompiler)
qt5_wrap_cpp(application_SOURCES ${application_HEADERS})
qt5_wrap_ui(application_SOURCES ${application_UIS})
qt5_add_resources(application_SOURCES mainrc.qrc)
qt5_add_resources(application_SOURCES qmlscatter.qrc)

# compile
add_executable(${PROJECT_NAME} WIN32 ${application_SOURCES} ${application_HEADERS})

# build it (link libraries)
target_link_libraries(${PROJECT_NAME} katricontrols Qt5::Core Qt5::Gui Qt5::Widgets Qt5::QuickWidgets Qt5::Qml Qt5::Quick)
install(TARGETS ${PROJECT_NAME} ${INSTALL_TARGETS_DEFAULT_ARGS})

include(QtGStreamerConfigCommon)

find_package(Qt5GStreamer REQUIRED)
include_directories(${QTGSTREAMER_INCLUDES})
find_package(GStreamer 1.2.0 COMPONENTS base)
find_package(GStreamerPluginsBase 1.2.0 COMPONENTS app video)
find_package(GLIB2)
find_package(GObject)
include_directories(
        ${GSTREAMER_INCLUDE_DIRS}
        ${GSTREAMER_APP_INCLUDE_DIR}
        ${GSTREAMER_BASE_INCLUDE_DIR}
        ${GSTREAMER_VIDEO_INCLUDE_DIR}
        ${GOBJECT_INCLUDE_DIR}
        ${GLIB2_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} ${GSTREAMER_APP_LIBRARY} ${GSTREAMER_BASE_LIBRARY} ${GOBJECT_LIBRARIES} ${GLIB2_LIBRARIES}
                      ${GSTREAMER_LIBRARY} ${GSTREAMER_VIDEO_LIBRARY} ${QTGSTREAMER_UI_LIBRARY}
)

# Find libarchive
#include(libarchiveConfig)
#FIND_PACKAGE(libarchive REQUIRED 3.4.0)
#if(NOT LibArchive_FOUND)
#    message(FATAL_ERROR "LibArchive is not found!")
#endif()
#include_directories(${LibArchive_INCLUDE_DIR})
#target_link_libraries(${PROJECT_NAME} ${LibArchive_LIBRARY})
#message("${LibArchive_INCLUDE_DIR} is found!")
#message("${LibArchive_LIBRARY} is found!")

#Install path prefix, prepended onto install directories.
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "..." FORCE)

if ("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "${CMAKE_SOURCE_DIR}/install/libkatricontrols.dll.a")
endif ()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
#    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "katricontrols/libkatricontrols.dll")
#    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "${CMAKE_INSTALL_PREFIX}/libkatricontrols.dll")
    if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/libkatricontrols.dll" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/libkatricontrols.dll")
        if (CMAKE_INSTALL_DO_STRIP)
            execute_process(COMMAND "C:/Qt/Qt5.13.0/Tools/mingw730_64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/libkatricontrols.dll")
        endif ()
    endif ()
endif ()
