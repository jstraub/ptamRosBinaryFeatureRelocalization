// Copyright 2008 Isis Innovation Limited
// This is the main extry point for PTAM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"

#include "Tracker.h"

using namespace std;
using namespace GVars3;

int main(int argc, char* argv[])
{
  cout << "  Welcome to PTAM " << endl;
  cout << "  --------------- " << endl;
  cout << "  Parallel tracking and mapping for Small AR workspaces" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2008 " << endl;  
  cout << endl;
  cout << "  Parsing settings.cfg ...." << endl;
  GUI.LoadFile("settings.cfg");
  
  //GUI.StartParserThread(); // Start parsing of the console input
  //atexit(GUI.StopParserThread);

  TrackerConfig trackerConfig;
  trackerConfig.framesBetweenManualReloc=21;
  trackerConfig.manualRelocalize=true;
  trackerConfig.pathToData=std::string("../../data/");
  trackerConfig.skipNFrames=0;
  trackerConfig.lastFrame=9999;
  trackerConfig.relocType=RelocType::PTAM;
  trackerConfig.featureType=BRIEF_T_HIST;

  cout<<"  ---------------- "<<endl;
  cout<<"  first argument: number of frames to skip until next one is handed to PTAM"<<endl;
  cout<<"  second argument: number of frames to between forced relocalization"<<endl;
  cout<<"  third argument: last frame that will be fed into PTAM"<<endl;
  cout<<"  ---------------- "<<endl;

  if(argc>1)
  {
    trackerConfig.skipNFrames=atoi(argv[1]);
  }
  if(argc>2)
  {
    trackerConfig.framesBetweenManualReloc=atoi(argv[2]);
    if(trackerConfig.framesBetweenManualReloc>0)
      trackerConfig.manualRelocalize=true;
    else
      trackerConfig.manualRelocalize=false;
  }
  if(argc>3)
  {
    trackerConfig.lastFrame=atoi(argv[3]);
  }
  if(argc>4)
  {
    uint32_t relocType=atoi(argv[4]);
    if(relocType<RelocType::NUMBER_RELOC_TYPES)
    { // only use this in case the value is plausible
      trackerConfig.relocType=relocType;
    }
  }
  if(argc>5)
  {
    trackerConfig.dtAfterRelocDone=double(atoi(argv[5]));
  }

  if(argc>6)
    {
      if(atoi(argv[6])<N_TYPES)
        trackerConfig.featureType=Feature_t(atoi(argv[6]));
      else{
        trackerConfig.featureType=BRIEF_T_HIST;
        cout<<"Feature type can only be 0(BRIEF), 1(ORB) or 2(FREAK)"<<endl;
      }
    }

  cout<<"  Skipping " <<trackerConfig.skipNFrames<<" frames before one is used."<<endl;
  cout<<"  Forcing relocalization every "<<trackerConfig.framesBetweenManualReloc<< " frames using relocalization-mechanism "<<trackerConfig.relocType<<endl;
  cout<<"    and I am waiting "<<trackerConfig.dtAfterRelocDone<<"ms after relocalisation has been completed, before tracking is resumed"<<endl;
  cout<<"  Last Frame: "<<trackerConfig.lastFrame<<endl;


#if defined WIN32 && (!defined WIN64 || defined EM64T) && \
    (_MSC_VER >= 1400 || defined CV_ICC) \
    || (defined __SSE2__ && defined __GNUC__ && __GNUC__ >= 4)
  #define CV_SSE2 1
  cout<<"CV_SSE2 defined"<<endl;
#else
  #define CV_SSE2 0
  cout<<"CV_SSE2 not defined"<<endl;
#endif

  try
  {
    System s(trackerConfig);
    s.Run();
  }
  catch(CVD::Exceptions::All e)
  {
    cout << endl;
    cout << "!! Failed to run system; got exception. " << endl;
    cout << "   Exception was: " << endl;
    cout << e.what << endl;
  }

  cout << "------ Finished System.Run() ------"<<endl;
  exit(0);
}










