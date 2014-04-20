// Copyright 2008 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

System::System(const TrackerConfig& trackerConfig) :
    mGLWindow(mVideoSource.Size(), "~PTAM~"), mConfig(trackerConfig)
{
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);

  mimFrameBW.resize(mVideoSource.Size());
  mimFrameRGB.resize(mVideoSource.Size());
  // First, check if the camera is calibrated.
  // If not, we need to run the calibration widget.
  Vector<NUMTRACKERCAMPARAMETERS> vTest;

  vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters",
      ATANCamera::mvDefaultParams, HIDDEN);
  mpCamera = new ATANCamera("Camera");
  Vector<2> v2;
  if (v2 == v2) ;
  if (vTest == ATANCamera::mvDefaultParams)
  {
    cout << endl;
    cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool"
        << endl;
    cout
        << "  and/or put the Camera.Parameters= line into the appropriate .cfg file."
        << endl;
    exit(1);
  }

  mpMap = new Map;
  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
  mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, *mpMap, *mpMapMaker,mConfig);
  mpARDriver = new ARDriver(*mpCamera, mVideoSource.Size(), mGLWindow);
  mpMapViewer = new MapViewer(*mpMap, mGLWindow);

  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("DrawAR=0");
  GUI.ParseLine("DrawMap=0");
  GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

  mbDone = false;

  // logging
  //fKalmanInput.open("../../data/kalmanInput.txt");
}
;


void System::Run()
{
  static bool initialized=false;
  int frameNr=0;
  uint32_t intraFrameCount=0;
  bool wroteZero=false;
  bool simTrackingLoss =false; // set by tracker if we want to loose tracking
  const uint32_t framesTooLoose=100; // about 10s break
  uint32_t framesLost=0;
  uint32_t noNewFrameCount=0;

  string pathToSeqOut(mConfig.pathToData);
  ofstream outSeq(pathToSeqOut.append("seqIdForSync.txt").c_str());

  while (!mbDone)
  {
    // We use two versions of each video frame:
    // One black and white (for processing by the tracker etc)
    // and one RGB, for drawing.

    // Grab new video frame...
    // visodo-AM: added imu data
    mVideoSource.GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB, mvOrientation,
        mvAngularVelocity, mvLinearAcceleration, mSec, mNsec, mSeq);

    /*
     * if no new frame has arrived (the timestamp did not update since the last time),
     * then don't go into the main loop
     */
    static bool bFirstFrame = true;
    if (bFirstFrame)
    {
      mpARDriver->Init();
      bFirstFrame = false;
    }


    if(mSeq==-1) // mSeq=-1 -> no frame received from ROS (at the beginning and at the end)
    { // at the beginning
      continue;
    }
    else if (mNsec != mOldNsec)
    {
      outSeq<<mSeq<<endl; // output the current sequence number in order to allow Matlab to syncronize
      if(mSeq>mConfig.lastFrame) break; // we are done now.
      cout<<" Image sequence #"<<mSeq<<endl;
      if(intraFrameCount<=mConfig.skipNFrames) intraFrameCount++; else intraFrameCount=0;
      if(intraFrameCount<mConfig.skipNFrames && initialized)
      {
        mOldNsec = mNsec;
        mOldSec = mSec;
        cout<< "Skipping Frame "<<intraFrameCount<<" !"<<endl;
        continue;
      }

      mGLWindow.SetupViewport();
      mGLWindow.SetupVideoOrtho();
      mGLWindow.SetupVideoRasterPosAndZoom();

      if (!mpMap->IsGood()) mpARDriver->Reset();

      static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN | SILENT);
      static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN | SILENT);

      bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
      bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;

      // control the number of frames to loos in the case of forced relocalization
      if(simTrackingLoss && framesLost<framesTooLoose)
        ++framesLost;
      else
        simTrackingLoss=false;

      if(simTrackingLoss)
      {
        cout<<"convolving #"<<frameNr<<" framesLost="<<framesLost<<endl;
        convolveGaussian(mimFrameBW, 10.0);
        Image<byte>::iterator itBw=mimFrameBW.begin();
        for(Image<Rgb<byte> >::iterator itRgb = mimFrameRGB.begin(); itRgb!=mimFrameRGB.end();itRgb++)
        {
          itRgb->blue=*itBw; itRgb->green=*itBw; itRgb->red=*itBw;
          itBw++;
        }
        mpARDriver->RenderImg(mimFrameRGB);
        mGLWindow.PrintString(ImageRef(240,240),"Relocalising",20.0);
      }else{
        // visodo-AM: give the tracker the imu information and store the timestamp
        mpTracker->TrackFrame(mimFrameBW, mvOrientation, mvAngularVelocity,
            mvLinearAcceleration, mSec, mNsec, mSeq, initialized, simTrackingLoss,
            !bDrawAR && !bDrawMap);
      }

      if (bDrawMap)
      {
        mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
        cout<<"render Map"<<endl;
      } else if (bDrawAR){
        mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose());
      }

      // mGLWindow.GetMousePoseUpdate();
      string sCaption;
      if (bDrawMap)
        sCaption = mpMapViewer->GetMessageForUser();
      else
        sCaption = mpTracker->GetMessageForUser();

      mGLWindow.DrawCaption(sCaption);
      mGLWindow.DrawMenus();
      mGLWindow.swap_buffers();
      mGLWindow.HandlePendingEvents();

      mOldNsec = mNsec;
      mOldSec = mSec;
      ++frameNr;
      noNewFrameCount=0;
    }
    else if(mSeq>100 && mNsec==mOldNsec) // current frame is the same as the previous one
    {
      if(noNewFrameCount>2500){ // received the same frame for 2500 times now (one time is pretty fast since there are no sleeps in this loop)
        mbDone=true; // exit this loop
      }else{
        ++noNewFrameCount; // keep counting no new frames up
      }
    }
  }

  outSeq.close();

  cout<<"Saving KeyFrame poses to file"<<endl;
  {// save KeyFrame poses -> they should be fully optimized now.
    std::ofstream keyPosesOut("../../data/allKeyFramePoses.txt");
    for(std::vector<KeyFrame*>::iterator it=mpMap->vpKeyFrames.begin();it!=mpMap->vpKeyFrames.end();it++)
      keyPosesOut<<(*it)->mSeq<<"\t"<<serializeForOutput((*it)->se3CfromW)<<endl;
    keyPosesOut.close();
  }

  cout<<"Saving all MapPoints with descriptors to file"<<endl;
  {// save briefFeature 3D and image positions alongside with the descriptor
    std::ofstream briefOut("../../data/allKeyBriefDescriptors.txt");
    for(std::vector<MapPoint*>::iterator it=mpMap->vpPoints.begin();it!=mpMap->vpPoints.end();it++)
      if((*it)!=NULL && (*it)->pBData!=NULL)
        briefOut<<(*it)->pBData->printForMatlab()<<endl;
    briefOut.close();
    // save all BRIEF observations of all MapPoints
    std::ofstream briefHistOut("../../data/allHistoryKeyBriefDescriptors.txt");
    uint32_t id=0;
    for(std::vector<MapPoint*>::iterator it=mpMap->vpPoints.begin();it!=mpMap->vpPoints.end();it++,++id)
      if((*it)!=NULL && (*it)->pBData!=NULL && (*it)->bData.size()>0)
        for(uint32_t i=0; i<(*it)->bData.size(); ++i)
        {
          // id represents the number of the map point that all these features are associated with
          briefHistOut<<id<<"\t"<<(*it)->bData[i]->printForMatlab()<<endl;
        };
    briefHistOut.close();
  }

  cout << "------ Stopping MapMaker ------"<<endl;
  delete mpMapMaker;
  cout << "------ Exiting System.Run() ------"<<endl;
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if (sCommand == "quit" || sCommand == "exit")
    static_cast<System*>(ptr)->mbDone = true;
}

