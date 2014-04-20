/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 
#ifndef BRIEFDATA_HPP_
#define BRIEFDATA_HPP_

#include "MapPoint.h"
#include "KeyFrame.h"
#include <BriefDesc.hpp>
#include <TooN/TooN.h>

#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

using namespace std;
using namespace TooN;

enum Feature_t{
  // full history of each map point
  BRIEF_T_HIST=0, // BRIEF on the scale of the detected FAST corner
  ORB_T_HIST=1, // ORB on the scale of the detected FAST corner
  FREAK_T_HIST=2, // FREAK on the scale of the detected FAST corner
  BRIEF_UNSCALED_T_HIST=3, // BRIEF on single scale.
  // only store last observation in map (update feature at each new observation of map point)
  BRIEF_T_UPDATE=10,
  ORB_T_UPDATE=11,
  FREAK_T_UPDATE=12,
  BRIEF_UNSCALED_T_UPDATE=13,
  N_TYPES=14
};

/* BriefData adds functionality needed by PTAM to the pure descriptor BriefDesc
 *
 */
class BriefData : public BriefDesc
{
public:
  BriefData(MapPoint* pMapPoint, KeyFrame* kF=NULL, Vector<2> p2=Vector<2>())
    : BriefDesc(), pt(*pMapPoint), mpKF(kF), v2p(p2)
  {
    pt.pBData=this;
  };
  BriefData(BriefData* bD)
    : BriefDesc(), pt(bD->pt), mpKF(bD->mpKF), v2p(bD->v2p)
  {
    pt.pBData=this;
  };

  virtual ~BriefData()
  {//    cout<<"BriefData::Destructor"<<endl;
  };

  MapPoint& pt; // Reference to the MapPoint which is associated with this BRIEF descriptor
  KeyFrame* mpKF; // in which KeyFrame was the BriefData extracted
  Vector<2> v2p; // where was the BRIEF extracted

  /* copy data from the other BriefDesc
   */
  virtual void copyDataFrom(BriefData* b)
  {
    memcpy(bd,b->bd,BRIEF_K);
    v2p=b->v2p;
    //mpKF=b->mpKF;
  };

  // TODO: this changed to brief0 ... brief32 now!
  // output in a format okay for matlab
  // x_w y_w z_w SeqNr u v brief0 ... brief8
  virtual std::string printForMatlab(void) const
  {
    uint32_t seq=0;
    if(mpKF!=NULL) seq=mpKF->mSeq;
    std::stringstream out;
    out<<pt.v3WorldPos[0]<<"\t"<<pt.v3WorldPos[1]<<"\t"<<pt.v3WorldPos[2]<<"\t";
    out<<seq<<"\t"<<v2p[0]<<"\t"<<v2p[1]<<"\t";
    for (size_t i=0; i<BRIEF_K/4; ++i)
    {
      uint32_t d=0;
      d=(uint32_t(bd[i*4])<<24) | (uint32_t(bd[i*4+1])<<16) | (uint32_t(bd[i*4+2])<<8) | (uint32_t(bd[i*4+3]));
      out<<d<<"\t";
    }
    return out.str();
  };

  // reads from an ifstream in the same matlab friendly format as it can print.
  virtual bool readFromMatlab(std::ifstream& in)
  {
    uint32_t dummySeq=0; // we cannot be sure that the Keyframe is here
    if(!(in>>pt.v3WorldPos[0]))
      return false;
    else{
      in>>pt.v3WorldPos[1]>>pt.v3WorldPos[2];
      in>>dummySeq>>v2p[0]>>v2p[1];
      for (size_t i=0; i<BRIEF_K; ++i)
        in>>bd[i];
      if(mpKF!=NULL) mpKF->mSeq=dummySeq;
      return true;
    }
  };

private:
  BriefData& operator=(const BriefData& brief);
};

class BriefDataS : public BriefData
{
public:
  BriefDataS(MapPoint* pMapPoint)
    : BriefData(pMapPoint)
  {
    bd=new uint8_t[BRIEF_K];
  };
  BriefDataS(MapPoint* pMapPoint, const uint8_t* bd_)
    : BriefData(pMapPoint)
  {
    bd=new uint8_t[BRIEF_K];
    memcpy(bd,bd_,BRIEF_K);
  };
  // copy constructor
  BriefDataS(BriefDataS* pBrief)
    : BriefData(pBrief)
  {
    bd=new uint8_t[BRIEF_K];
    memcpy(bd,pBrief->bd,BRIEF_K);
    cout<<"BriefData: copy constructor! "<<endl;
  };

  virtual ~BriefDataS()
  {
    delete bd;
  }
private:
  BriefDataS& operator=(const BriefDataS& brief);
};

/* like BriefData but allows the use of descriptors that are managed by someone else
 *
 */
class BriefDataM : public BriefData
{
public:
  // constructor in case the briefdata is managed by someone else
  BriefDataM(MapPoint* mp, uint8_t* bd_)
    : BriefData(mp)
  {
    bd=bd_;
  };
  BriefDataM(MapPoint* mp, KeyFrame* kF, uint8_t* bd_)
  : BriefData(mp,kF)
  {
    bd=bd_;
  };
  BriefDataM(MapPoint* mp, KeyFrame* kF, Vector<2> p2, uint8_t* bd_)
  : BriefData(mp,kF,p2)
  {
    bd=bd_;
  };
  virtual ~BriefDataM()
  {};

private:
  BriefDataM& operator=(const BriefDataM& brief);
};

//// load BRIEF descriptors of keypoints seen in keyFrames
bool loadBriefs(const std::string pathToData,
    const std::string fileName,
    std::vector<BriefData*>& briefs,
    std::map<BriefData*,size_t>& briefFrames);

bool loadHistoryBriefs(const std::string pathToData,
    const std::string fileName,
    std::vector<std::vector<BriefData*> >& briefs,
    std::map<BriefData*,size_t>& briefFrames);

// used in testLSHRansacLoc.cpp to allow testing of full history vs updating
bool loadHistoryBriefs(const string pathToData,
    const string fileName,
    std::map<BriefData*,uint32_t>& briefFrames,
        std::map<uint32_t,vector<MapPoint*> >& frames);

/* Load the HistoryBriefs in a way that make sit easy to access the
 * features observed at a specific keyFrame
 *
 * Observations of the same keypoint are treated equally and just added
 * to the frame in which they were observed.
 *
 * Good for testing relocalization without much knowledge from
 * in front of a given keyframe
 */
bool loadFrameBriefs(const std::string pathToData,
    const std::string fileName,
    std::map<BriefData*,uint32_t>& briefFrames,
    std::map<uint32_t,vector<BriefData*> >& frames);


//std::ostream& operator<< (std::ostream &out, const BriefData& a);
//std::ostream& operator<< (std::ostream &out, const BriefDataM& a);


#endif /* BRIEFDATA_HPP_ */
