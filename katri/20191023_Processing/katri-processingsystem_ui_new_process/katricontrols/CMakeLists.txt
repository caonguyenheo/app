project ( katricontrols )

include(GenerateExportHeader)

# declaring files in your project
set ( katricontrols_SOURCES
    common.cpp
    simulationview.cpp
    viewerpanel.cpp
    playbackcontrol.cpp
    navigationpanel.cpp
    exportdata.cpp
    exportdata_left.cpp
    viewdataexport.cpp
    backpanel.cpp
    lidar/scatterdatamodifier.cpp
    lidar/lidarview.cpp
    lidar/simulationlidarview.cpp
    imageview.cpp
    canbusview.cpp
    MapViewer/osm/Geometry.cpp
    MapViewer/osm/GeometryLineString.cpp
    MapViewer/osm/GeometryPoint.cpp
    MapViewer/osm/GeometryPointArrow.cpp
    MapViewer/osm/GeometryPointCircle.cpp
    MapViewer/osm/GeometryPointImage.cpp
    MapViewer/osm/GeometryPointImageScaled.cpp
    MapViewer/osm/GeometryPointShape.cpp
    MapViewer/osm/GeometryPointShapeScaled.cpp
    MapViewer/osm/GeometryPolygon.cpp
    MapViewer/osm/GeometryPolygonImage.cpp
    MapViewer/osm/GeometryWidget.cpp
    MapViewer/osm/GPS_Position.cpp
    MapViewer/osm/ImageManager.cpp
    MapViewer/osm/Layer.cpp
    MapViewer/osm/LayerGeometry.cpp
    MapViewer/osm/LayerMapAdapter.cpp
    MapViewer/osm/MapAdapter.cpp
    MapViewer/osm/MapAdapterGoogle.cpp
    MapViewer/osm/MapAdapterOpenSeaMap.cpp
    MapViewer/osm/MapAdapterOSM.cpp
    MapViewer/osm/MapAdapterTile.cpp
    MapViewer/osm/MapAdapterWMS.cpp
    MapViewer/osm/MapAdapterYahoo.cpp
    MapViewer/osm/NetworkManager.cpp
    MapViewer/osm/Projection.cpp
    MapViewer/osm/ProjectionEquirectangular.cpp
    MapViewer/osm/ProjectionSphericalMercator.cpp
    MapViewer/osm/QMapControl.cpp
    MapViewer/osm/QProgressIndicator.cpp
    mapviewer.cpp
    glwidgetimage.cpp
)

set ( katricontrols_HEADERS
    common.h
    simulationview.h
    viewerpanel.h
    playbackcontrol.h
    navigationpanel.h
    exportdata.h
    exportdata_left.h
    viewdataexport.h
    backpanel.h
    lidar/scatterdatamodifier.h
    lidar/lidarview.h
    lidar/simulationlidarview.h
    imageview.h
    canbusview.h
    MapViewer/osm/Geometry.h
    MapViewer/osm/GeometryLineString.h
    MapViewer/osm/GeometryPoint.h
    MapViewer/osm/GeometryPointArrow.h
    MapViewer/osm/GeometryPointCircle.h
    MapViewer/osm/GeometryPointImage.h
    MapViewer/osm/GeometryPointImageScaled.h
    MapViewer/osm/GeometryPointShape.h
    MapViewer/osm/GeometryPointShapeScaled.h
    MapViewer/osm/GeometryPolygon.h
    MapViewer/osm/GeometryPolygonImage.h
    MapViewer/osm/GeometryWidget.h
    MapViewer/osm/GPS_Position.h
    MapViewer/osm/ImageManager.h
    MapViewer/osm/Layer.h
    MapViewer/osm/LayerGeometry.h
    MapViewer/osm/LayerMapAdapter.h
    MapViewer/osm/MapAdapter.h
    MapViewer/osm/MapAdapterGoogle.h
    MapViewer/osm/MapAdapterOpenSeaMap.h
    MapViewer/osm/MapAdapterOSM.h
    MapViewer/osm/MapAdapterTile.h
    MapViewer/osm/MapAdapterWMS.h
    MapViewer/osm/MapAdapterYahoo.h
    MapViewer/osm/NetworkManager.h
    MapViewer/osm/Projection.h
    MapViewer/osm/ProjectionEquirectangular.h
    MapViewer/osm/ProjectionSphericalMercator.h
    MapViewer/osm/QMapControl.h
    MapViewer/osm/QProgressIndicator.h
    MapViewer/osm/m_constants.h
    MapViewer/osm/Point.h
    mapviewer.h
    glwidgetimage.h
)

set ( katricontrols_UIS
    simulationview.ui
    lidar/simulationlidarview.ui
    viewerpanel.ui
    playbackcontrol.ui
    navigationpanel.ui
    exportdata.ui
    exportdata_left.ui
    viewdataexport.ui
    backpanel.ui
    imageview.ui
    canbusview.ui
    mapviewer.ui
)

# Next lines needed for building all Qt projects
FIND_PACKAGE(Qt5Core REQUIRED)
FIND_PACKAGE(Qt5Gui REQUIRED)
FIND_PACKAGE(Qt5Widgets REQUIRED)
FIND_PACKAGE(Qt5QuickWidgets REQUIRED)
FIND_PACKAGE(Qt5DataVisualization REQUIRED)

add_definitions( ${QT_DEFINITIONS} )
include_directories(
    ${CMAKE_SOURCE_DIR}/katricontrols
    ${CMAKE_BINARY_DIR}/katricontrols
    ${CMAKE_BINARY_DIR}/katricontrols/videoplayer
)

# using Qt meta-system (precompiler)
qt5_wrap_cpp(katricontrols_SOURCES ${katricontrols_HEADERS})
qt5_wrap_ui(katricontrols_SOURCES ${katricontrols_UIS})
qt5_add_resources(katricontrols_SOURCES librc.qrc)
qt5_add_resources(application_SOURCES qmlscatter.qrc)
# compile
add_library(${PROJECT_NAME} SHARED ${katricontrols_SOURCES} ${katricontrols_HEADERS})
generate_export_header(${PROJECT_NAME} BASE_NAME katricontrols)

# build it (link libraries)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::QuickWidgets Qt5::DataVisualization)
install(TARGETS ${PROJECT_NAME} ${INSTALL_TARGETS_DEFAULT_ARGS})

include(QtGStreamerConfigCommon)

find_package(Qt5GStreamer REQUIRED)
include_directories(${QTGSTREAMER_INCLUDES})
find_package(GStreamer 1.2.0 COMPONENTS base)
find_package(GStreamerPluginsBase 1.2.0 COMPONENTS app video)
find_package(GLIB2)
find_package(GObject)
find_package(Qt5 REQUIRED COMPONENTS Quick Qml)
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
                      Qt5::Qml Qt5::Quick
)
