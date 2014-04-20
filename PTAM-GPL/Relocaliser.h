// -*- c++ -*- 
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage-based relocaliser
// 
// Each KF stores a small, blurred version of itself;
// Just compare a small, blurred version of the input frame to all the KFs,
// choose the closest match, and then estimate a camera rotation by direct image
// minimisation.

#ifndef __RELOCALISER_H
#define __RELOCALISER_H
#include <TooN/se2.h>
#include "ATANCamera.h"
#include "SmallBlurryImage.h"
#include "timer.hpp"
#include <results.hpp>

#include "Map.h"


class Relocaliser
{
public:
  Relocaliser(ATANCamera &camera)
    : mCamera(camera)
  {};
  virtual ~Relocaliser()
  {};

  bool AttemptRecovery(KeyFrame &k, float& dt)
  {
    Timer t0;
    bool success=AttemptRecovery(k);
    dt=t0.toc();
    cout<<"=== Relocalizer dt="<<dt<<"ms ==="<<endl;
    return success;
  }

  virtual bool AttemptRecovery(KeyFrame &k)
  {return false;}

  SE3<> BestPose()
  {
    return mse3Best;
  };

  const LocResult& getRelocResults(void)
  {
    return mRes;
  };

protected:

  ATANCamera mCamera;
  SE3<> mse3Best; // this pose is cTw
  LocResult mRes;
};

class PTAMRelocaliser : public Relocaliser
{
public:
  PTAMRelocaliser(Map &map, ATANCamera &camera)
    : Relocaliser(camera), mMap(map)

  {};
  ~PTAMRelocaliser()
  {};
  bool AttemptRecovery(KeyFrame &k);

protected:
  void ScoreKFs(KeyFrame &kCurrentF);
//  Map &mMap;
//  ATANCamera mCamera;
  Map &mMap;
  int mnBest;
  double mdBestScore;
  SE2<> mse2;
//  SE3<> mse3Best;

};

#endif









