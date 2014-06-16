cmake_minimum_required(VERSION 2.8.3)
project(videre_stereo_cam)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport  stereo_image_proc diagnostic_updater stereo_msgs dynamic_reconfigure driver_base tf cv_bridge)
find_package(Boost REQUIRED COMPONENTS thread signals)


generate_dynamic_reconfigure_options(
  cfg/VidereStereoCam.cfg
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp sensor_msgs image_transport  stereo_image_proc diagnostic_updater stereo_msgs dynamic_reconfigure driver_base tf cv_bridge
  DEPENDS
	Boost
)
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

#ADD_DEFINITIONS(-DHAVE_LIBDV)
#add_executable(arm5_grabber src/main.cc src/dvframe.cc  src/error.cc  src/frame.cc  src/hdvframe.cc  src/iec13818-1.cc  src/iec13818-2.cc  src/ieee1394io.cc  src/raw1394util.c)
#target_link_libraries(arm5_grabber avc1394 iec61883 raw1394 rom1394 pthread dv ${catkin_LIBRARIES})


# auto-generated dynamic reconfiguration GUI
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

#rosbuild_add_boost_directories()

include(FindPkgConfig)
if (PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)
   pkg_check_modules(GTK gtk+-2.0)
   if (GTK_FOUND)
      include_directories(${GTK_INCLUDE_DIRS})
      link_directories(${GTK_LIBRARY_DIRS})
      add_definitions(-DHAVE_GTK)
   endif(GTK_FOUND)
endif(PKG_CONFIG_FOUND AND UNIX AND NOT APPLE)

add_executable(videre_stereo_node src/videre_stereo_cam/stereoimage.cpp
                                           src/videre_stereo_cam/videre_stereo_1394.cpp
                                           src/videre_stereo_node.cpp)
set_target_properties(videre_stereo_node PROPERTIES COMPILE_FLAGS "-msse2 -mpreferred-stack-boundary=4")
target_link_libraries(videre_stereo_node dc1394 ${catkin_LIBRARIES})
add_dependencies(videre_stereo_node ${PROJECT_NAME}_gencfg)


#add_executable(disparity_view src/disparity_view.cpp)
#target_link_libraries(disparity_view ${GTK_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#Cannot compile disparity_view before changin from old CvBridge to new cv_bridge functions...
# Old API: http://devel.iri.upc.edu/docs/roswiki/docs/diamondback/api/cv_bridge/html/c++/classsensor__msgs_1_1CvBridge.html
# tutorial: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
# 

install(
  TARGETS
    videre_stereo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

