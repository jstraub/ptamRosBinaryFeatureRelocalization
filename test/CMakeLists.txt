#/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
# * Licensed under the GPLv3 license. See the license file LICENSE.
# * 
# * If this code is used, the following should be cited:  
# * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
# * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
# * IEEE International Conference on Image Processing (ICIP), 2013 
# */

project(tests)
# States that CMake required version must be >= 2.6
cmake_minimum_required(VERSION 2.6)

set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE debug)

if( $ENV{HOME} STREQUAL "/home/jstraub" )
  set(ON_NAVVIS "")  
else()  
  set(ON_NAVVIS "-DON_NAVVIS")  
endif()

# add compiler flags
set(CMAKE_CXX_FLAGS_RELEASE "-g -Wall -O2 -finline-functions ${ON_NAVVIS} -DHAVE_TOON -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${ON_NAVVIS} -DHAVE_TOON")
 
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
 message("OPENMP FOUND")
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Include directories (.h .hpp files)
include_directories(
  .
  /usr/include/opencv-2.3.1/ #for navvis
  /usr/local/include
  /opt/ros/groovy/include
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/PTAM-GPL
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/lshKBM/include    
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/briefRelocalize/
  $ENV{NAVVIS_WORKSPACE}/3rdparty/libcvd
  $ENV{NAVVIS_WORKSPACE}/3rdparty/gvars3
  $ENV{NAVVIS_WORKSPACE}/3rdparty
)
# Include library directories
link_directories(
  /usr/local/lib/
  /opt/ros/groovy/lib
  $ENV{NAVVIS_WORKSPACE}/3rdparty/libcvd
  $ENV{NAVVIS_WORKSPACE}/3rdparty/gvars3
)
# define the sources
set(SRC 
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/lshKBM/src/BriefDesc.cpp
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/briefRelocalize/BriefData.cpp
)
# define libraries
set(BOOST boost_unit_test_framework boost_thread)
set(PTAM GVars3 cvd)

add_executable(testBRIEF test_BRIEF.cpp ${SRC})
target_link_libraries(testBRIEF ${BOOST} ${PTAM} stdc++)

add_executable(test_BriefNN test_BriefNN.cpp ${SRC})
target_link_libraries(test_BriefNN ${BOOST} ${PTAM} stdc++)

add_executable(testRANSAC testRANSAC.cpp 
  ${SRC}
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/briefRelocalize/Ransac.cpp
  $ENV{NAVVIS_WORKSPACE}/ptamBriefRelocalizeStandalone/PTAM-GPL/ATANCamera.cc
) 
target_link_libraries(testRANSAC ${BOOST} ${PTAM} stdc++)

