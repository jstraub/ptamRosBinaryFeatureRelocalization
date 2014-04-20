//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"
#include "BriefRelocalizer.hpp"
#include "outputHelper.hpp"
#include "Random.hpp"

#include <cvd/image_io.h>

#include <sstream>
#include <fstream>
#include <vector>
#include <list>
#include <stdint.h>
#include <stdio.h>

class TrackerData;

struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};

struct RelocType{
  static const uint32_t PTAM=0;
  static const uint32_t BRIEF_LSH=1;
  static const uint32_t BRIEF_KBINMEANS=2;
  static const uint32_t NUMBER_RELOC_TYPES=3;
};

struct TrackerConfig
{
  // to inject manual relocalizations
  bool manualRelocalize;
  int framesBetweenManualReloc;
  double dtAfterRelocDone; // time to wait after succcessfull relocalisation [ms]
  int lastFrame; // number of frames to work on at maximum
  uint32_t relocType; // which relocalization Method to use
  // where to store recorded data
  std::string pathToData;
  // skip frames
  uint32_t skipNFrames;
  // feature type to use: 0=BRIEF; 1=ORB; 2=FREAK
  Feature_t featureType;
};

class Tracker
{
public:
  Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm, const TrackerConfig& config);
  ~Tracker();
  
  // TrackFrame is the main working part of the tracker: call this every frame.
  // visodo-AM: add imu data
  void TrackFrame(CVD::Image<CVD::byte> &imFrame,
		  std::vector<double> &vOrientation, std::vector<double> &vVelocity,
		  std::vector<double> &vAcceleration, int32_t &t_sec, int32_t &t_nsec,
		  int32_t& seq, bool& initialized, bool& simTrackingLoss, bool bDraw);

  inline SE3<> GetCurrentPose() {return mse3CamFromWorld;}
  
  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();
  
protected:
  KeyFrame mCurrentKF;            // The current working frame as a keyframe struct
  
  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  ATANCamera mCamera;             // Projection model
  vector<Relocaliser*> mRelocalisers;  // Relocalisation module

  CVD::ImageRef mirSize;          // Image size of whole image
  
  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.
  void RenderGrid();              // Draws the reference grid

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
  void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
  void TrackForInitialMapAuto();  // visodo-AM: automatic initialization
  enum {TRAIL_TRACKING_NOT_STARTED, 
	TRAIL_TRACKING_STARTED, 
	TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
  void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
  int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
  std::list<Trail> mlTrails;      // Used by trail tracking
  KeyFrame mFirstKF;              // First of the stereo pair
  KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches
  
  // Methods for tracking the map once it has been made:
  void TrackMap();                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<TrackerData*> &vTD, 
		      int nRange, 
		      int nFineIts);  // Finds points in the image
  Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD, 
			   double dOverrideSigma = 0.0, 
			   bool bMarkOutliers = false); // Updates pose from found points.
  SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
  SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
  Vector<6> mv6CameraVelocity;    // Motion model
  double mdVelocityMagnitude;     // Used to decide on coarse tracking 
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?
  
  bool mbDraw;                    // Should the tracker draw anything to OpenGL?
  
  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
  void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe
  
  // Tracking quality control:
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;
  int mNumFramesForInitialTracking; // visodo-AM: count frames that should be considered for automatic initialization
  
  // Relocalisation functions:
  bool AttemptRecovery(float& dt);         // Called by TrackFrame if tracking is lost.
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  Vector<6> mv6SBIRot;
  bool mbUseSBIInit;
  
  // User interaction for initial tracking:
  bool mbUserPressedSpacebar;
  std::ostringstream mMessageForUser;
  
  // GUI interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;

  // Data recording
  std::ofstream mfKeyFrameIDs; // Sequence numbers for all KeyFrames
  std::ofstream mfRelocalizationFrameNr; // Sequence numbers of all relocalization frames
  std::ofstream mfFramePoses; // Outfile for pose at each frame (via sequence numbers)
  std::ofstream mfFrame2Seq; // mapping between internal frame numbers and sequence numbers from PTAM
  std::ofstream mfBriefDescriptors; // BRIEF descriptors of non-KeyFrame frames.
  std::ofstream mfFeatureStats; // how many features are extracted per pyramid level per frame?
  int32_t mSeq; // sequence number from ROS
  int32_t mFirstKFSeq; // Sequence number of the first KeyFrame

  // config of the tracker -> manual relocalization and paths to the datafiles
  TrackerConfig mConfig;

  // Random number generator seeded in constructor
  static const int RND_SEED=1;
  Random mRand;

};


#endif




