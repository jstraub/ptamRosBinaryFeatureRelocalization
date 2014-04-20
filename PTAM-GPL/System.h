// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "VideoSource.h"
#include "GLWindow2.h"

#include <cvd/image.h>
#include <cvd/convolution.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <stdint.h>
#include <fstream>

#include "Tracker.h"
#include "outputHelper.hpp"

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class ARDriver;
class MapViewer;

class System
{
public:
  System(const TrackerConfig& trackerConfig);
  void Run();
  
private:
  VideoSource mVideoSource;
  GLWindow2 mGLWindow;
  CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;
  CVD::Image<CVD::byte> mimFrameBW;
  
  // visodo-AM: added vectors containing imu data
  std::vector<double> mvOrientation; // x,y,z,w
  std::vector<double> mvAngularVelocity; // x,y,z
  std::vector<double> mvLinearAcceleration; // x,y,z
  int32_t mSec; // ROS timestamp seconds
  int32_t mOldSec; // old value
  int32_t mNsec; // ROS timestamp nanoseconds
  int32_t mOldNsec; // old value to compare if new frame has arrived
  int32_t mSeq; // Sequenze ID of the current callback data

  Map *mpMap; 
  MapMaker *mpMapMaker; 
  Tracker *mpTracker; 
  ATANCamera *mpCamera;
  ARDriver *mpARDriver;
  MapViewer *mpMapViewer;
  
  bool mbDone;

  // percentage skipped: SKIP_N_FRAMES/(SKIP_N_FRAMES+1)
  const static double SKIP_N_FRAMES=0;

  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

  // logging
  std::ofstream fKalmanInput;

  TrackerConfig mConfig;
};



#endif
