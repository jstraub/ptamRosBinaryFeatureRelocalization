#/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
# * Licensed under the GPLv3 license. See the license file LICENSE.
# * 
# * If this code is used, the following should be cited:  
# * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
# * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
# * IEEE International Conference on Image Processing (ICIP), 2013 
# */

# Project name is not mandatory
project(tests)
# States that CMake required version must be >= 2.6
cmake_minimum_required(VERSION 2.6)

set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Profiling)

if( $ENV{HOME} STREQUAL "/home/straub" )
  set(ON_NAVVIS "")
  message("Compiling for laptop")
else()
  set(ON_NAVVIS "-DON_NAVVIS")
  message("Compiling for NAVVIS")
endif()

# add compiler flags
set(CMAKE_CXX_FLAGS_RELEASE "-g -Wall -O2 -finline-functions ${ON_NAVVIS} -DNDEBUG")
set(CMAKE_CXX_FLAGS_PROFILING "-pg -g ${ON_NAVVIS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${ON_NAVVIS}")

# TooN is necessary!
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_TOON")

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
 message("OPENMP FOUND")
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Include directories (.h .hpp files)
include_directories(
  $ENV{NAVVIS_WORKSPACE}/briefRelocalize # for TooN
  include
  /usr/local/include
  /opt/ros/fuerte/include/
  /usr/include/opencv-2.3.1/ #for navvis
)
# Include library directories
link_directories(
    /build
    /usr/lib/
    /usr/local/lib/   
    /opt/ros/fuerte/lib
)

# define libraries
set(BOOST boost_unit_test_framework boost_thread)
set(CBIR_LIB opencv_core)
set(KBM_LIB stdc++)
if( $ENV{HOME} STREQUAL "/home/straub" )
  set(KBM_LIB ${KBM_LIB} tinyxml)  
else()  
  set(SRC_KBM ${SRC_KBM}
    $ENV{NAVVIS_WORKSPACE}/3rdparty/tinyxml/tinystr.cpp  
    $ENV{NAVVIS_WORKSPACE}/3rdparty/tinyxml/tinyxml.cpp 
    $ENV{NAVVIS_WORKSPACE}/3rdparty/tinyxml/tinyxmlerror.cpp  
    $ENV{NAVVIS_WORKSPACE}/3rdparty/tinyxml/tinyxmlparser.cpp)
endif()

add_executable(testLshTable tests/testLshTable.cpp)
target_link_libraries(testLshTable ${BOOST} stdc++)

add_executable(test_Random tests/test_Random.cpp)
target_link_libraries(test_Random ${BOOST} stdc++)

#add_executable(testBriefDesc tests/testBriefDesc.cpp
#  src/BriefDesc.cpp)
#target_link_libraries(testBriefDesc ${BOOST} stdc++)



