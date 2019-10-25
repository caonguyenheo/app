project ( katricontrols )

include(GenerateExportHeader)

# declaring files in your project
set ( katricontrols_SOURCES
    common/common.cpp
    common/backpanel.cpp
    viewerpanel.cpp
    controll.cpp
    recordbutton.cpp    
    controlpanel.cpp
    graph/qcustomplot.cpp
    graph/customgraph.cpp
    graph/customgraphsmall.cpp
    graph/performancemanager.cpp
    graph/navigationmanager.cpp
    graph/navigationmanager.cpp
    graph/navigationmanagersmall.cpp
    videoplayer/playbackcontrol.cpp
    videoplayer/fullscreen.cpp
    videoplayer/video.cpp
    videoplayer/zoombutton.cpp
    videoplayer/customgraphicsview.cpp
    camera/viewcommand.cpp
    camera/viewerperformance.cpp
    camera/viewcamera.cpp
    camera/viewdatacamera.cpp
    menubar.cpp
    common/FrameLess.cpp
    message/singlemessage.cpp
    ui_all/record.cpp
    katristyle/katristyle.cpp
    checkedbox/checkedbox.cpp
)

set ( katricontrols_HEADERS
    common/common.h
    common/backpanel.h
    viewerpanel.h
    controll.h
    recordbutton.h
    controlpanel.h
    graph/qcustomplot.h
    graph/customgraph.h
    graph/customgraphsmall.h
    graph/performancemanager.h
    graph/navigationmanager.h
    graph/navigationmanagersmall.h
    videoplayer/playbackcontrol.h
    videoplayer/fullscreen.h
    videoplayer/video.h
    videoplayer/zoombutton.h
    videoplayer/customgraphicsview.h
    camera/viewcommand.h
    camera/viewerperformance.h
    camera/viewcamera.h
    camera/viewdatacamera.h
    menubar.h
    common/FrameLess.h
    message/singlemessage.h
    ui_all/record.h
    katristyle/katristyle.h
    checkedbox/checkedbox.h
)

set ( katricontrols_UIS
    common/backpanel.ui
    viewerpanel.ui
    controll.ui
    recordbutton.ui
    controlpanel.ui
    graph/customgraph.ui
    graph/customgraphsmall.ui
    graph/performancemanager.ui
    graph/navigationmanager.ui
    graph/navigationmanagersmall.ui
    videoplayer/playbackcontrol.ui
    videoplayer/zoombutton.ui
    camera/viewcommand.ui
    camera/viewerperformance.ui
    camera/viewcamera.ui
    camera/viewdatacamera.ui
    menubar.ui
    ui_all/record.ui
)

# Next lines needed for building all Qt projects
FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(Qt5Gui REQUIRED)
FIND_PACKAGE(Qt5Widgets REQUIRED)

add_definitions( ${QT_DEFINITIONS} )
include_directories(
    ${CMAKE_SOURCE_DIR}/katricontrols
    ${CMAKE_BINARY_DIR}/katricontrols
)

# using Qt meta-system (precompiler)
qt5_wrap_cpp(katricontrols_SOURCES ${katricontrols_HEADERS})
qt5_wrap_ui(katricontrols_SOURCES ${katricontrols_UIS})
qt5_add_resources(katricontrols_SOURCES resources.qrc)

# compile
add_library(${PROJECT_NAME} SHARED ${katricontrols_SOURCES} ${katricontrols_HEADERS})
generate_export_header(${PROJECT_NAME} BASE_NAME katricontrols)

# build it (link libraries)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION bin
        ARCHIVE DESTINATION bin)
