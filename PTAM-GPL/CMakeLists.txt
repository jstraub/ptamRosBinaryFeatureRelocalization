#/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
# * Licensed under the GPLv3 license. See the license file LICENSE.
# * 
# * If this code is used, the following should be cited:  
# * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
# * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
# * IEEE International Conference on Image Processing (ICIP), 2013 
# */

project(PTAM)
cmake_minimum_required(VERSION 2.6)


set(CMAKE_MODULE_PATH $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/PTAM-GPL ${CMAKE_MODULE_PATH})
message("CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Profiling)

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -finline-functions -D_LINUX -D_REENTRANT -DTIME_MEASUREMENT -Wall -march=nocona -msse3")
set(CMAKE_CXX_FLAGS_PROFILING "-pg")

FIND_PACKAGE(GLIB2 REQUIRED)
if(OPENMP_FOUND)
   message("glib-2.0 FOUND")
else()
  message("glib-2.0 NOT FOUND")
  set(GLIB2_LIBRARIES "-lGL")
endif()
FIND_PACKAGE(GStreamer REQUIRED)
if(OPENMP_FOUND)
  message("gstreamer FOUND")
  message("${GLIB2_LIBRARIES}")
  message("${GSTREAMER_INCLUDE_DIR}")
  message("${GSTREAMER_DEFINITIONS}")
else()
  message("gstreamer NOT FOUND")
endif()

include_directories(
  /usr/local/include 
  /opt/ros/groovy/include/
  $ENV{NAVVIS_WORKSPACE}/3rdparty/gvars3 
  $ENV{NAVVIS_WORKSPACE}/3rdparty/libcvd
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/lshKBM/include 
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/briefRelocalize/
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/PTAM-GPL
  ${GLIB2_INCLUDE_DIR}
  ${GSTREAMER_INCLUDE_DIR}
)

link_directories(
  /usr/lib
  /usr/local/lib 
  /opt/ros/groovy/lib
  $ENV{NAVVIS_WORKSPACE}/3rdparty/libcvd
  $ENV{NAVVIS_WORKSPACE}/3rdparty/gvars3
)

set(VIDEOSOURCE
  #VideoSource_Linux_DV.cc
  #VideoSource_Linux_Gstreamer_File.cc
  VideoSource_Linux_ROS.cc
)
set(SRC 
  main.cc
  GLWindow2.cc
  GLWindowMenu.cc
  ${VIDEOSOURCE}
  System.cc
  ATANCamera.cc
  KeyFrame.cc
  MapPoint.cc
  Map.cc
  SmallBlurryImage.cc
  ShiTomasi.cc
  HomographyInit.cc
  MapMaker.cc
  Bundle.cc
  PatchFinder.cc
  Relocaliser.cc
  MiniPatch.cc
  MapViewer.cc
  ARDriver.cc
  EyeGame.cc
  Tracker.cc
  ../briefRelocalize/BriefExtractor.cpp
  ../briefRelocalize/BriefRelocalizer.cpp
  ../briefRelocalize/outputHelper.cpp
  ../briefRelocalize/Ransac.cpp
  ../lshKBM/src/BriefDesc.cpp
)


set(CALIB_SRC
  GLWindow2.cc
  GLWindowMenu.cc
  ${VIDEOSOURCE}
  CalibImage.cc
  CalibCornerPatch.cc
  ATANCamera.cc
  CameraCalibrator.cc
)

#`pkg-config --cflags glib-2.0` `pkg-config --cflags gstreamer-0.10`    
#`pkg-config --libs glib-2.0` `pkg-config --libs gstreamer-app-0.10`
set(LIBS GVars3 cvd png lapack opencv_core opencv_imgproc opencv_features2d opencv_highgui)

add_executable(PTAM main.cc ${SRC})
target_link_libraries(PTAM ${LIBS} ${GLIB2_LIBRARIES} ${GSTREAMER_LIBRARIES})
install(TARGETS PTAM DESTINATION $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/PTAM-GPL COMPONENT PTAM)

add_executable(CameraCalibrator CameraCalibrator.cc ${CALIB_SRC})
target_link_libraries(CameraCalibrator ${LIBS} ${GLIB2_LIBRARIES} ${GSTREAMER_LIBRARIES})
install(TARGETS CameraCalibrator DESTINATION $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/PTAM-GPL COMPONENT CameraCalibrator)

