// Copyright 2008 Isis Innovation Limited
#include "Relocaliser.h"
#include "SmallBlurryImage.h"
#include <cvd/utility.h>
#include <gvars3/instances.h>

using namespace CVD;
using namespace std;
using namespace GVars3;


bool PTAMRelocaliser::AttemptRecovery(KeyFrame &kCurrent)
{
  // Ensure the incoming frame has a SmallBlurryImage attached
  if(!kCurrent.pSBI)
    kCurrent.pSBI = new SmallBlurryImage(kCurrent); // sigma =2.5
  else
    kCurrent.pSBI->MakeFromKF(kCurrent);
  
  // Find the best ZMSSD match from all keyframes in map -> argmin ZMSSD(current_frame, keyframes)
  ScoreKFs(kCurrent);

  // And estimate a camera rotation from a 3DOF image alignment
  // Do ESM to find SE2 rotation
  pair<SE2<>, double> result_pair = kCurrent.pSBI->IteratePosRelToTarget(*mMap.vpKeyFrames[mnBest]->pSBI, 6);
  mse2 = result_pair.first;
  double dScore =result_pair.second;
  
  // get pose of the closest keyFrame
  SE3<> se3KeyFramePos = mMap.vpKeyFrames[mnBest]->se3CfromW;
  // compute the best guess of the current pose by updating the
  // keyframe pose with the SE3 obtained from the SE2 from the ESM.
  mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera) * se3KeyFramePos;
  
  if(dScore < GV2.GetDouble("Reloc2.MaxScore", 9e6, SILENT)) // visodo-AM: initially 9e6
    return true;
  else 
    return false;
};

// Compare current KF to all KFs stored in map by
// Zero-mean SSD
void PTAMRelocaliser::ScoreKFs(KeyFrame &kCurrent)
{
  mdBestScore = 99999999999999.9;
  mnBest = -1;
  
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      double dSSD = kCurrent.pSBI->ZMSSD(*mMap.vpKeyFrames[i]->pSBI);
      if(dSSD < mdBestScore)
	{
	  mdBestScore = dSSD;
	  mnBest = i;
	}
    }
}

