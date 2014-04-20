/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef PTAMRELOC_HPP_
#define PTAMRELOC_HPP_

#include <BriefRelocalizer.hpp>
//#include <briefIR.hpp>
#include <timer.hpp>
#include <TooN/TooN.h>

#include <vector>

using namespace std;
using namespace TooN;

template <typename RANSAC, class Mod, class Desc>
class PtamReLoc : public SingleThread<LocResult>
{
public:
  PtamReLoc( Mod* model,
      LSH<Desc,uint32_t>* classifier,
      const vector<Desc*>& queryMPs,         // make copy
      const RansacParams& ransacParams, double width=640, double height= 480, uint32_t kNN=1)
  :   SingleThread<LocResult>(),
      mQueryMPs(queryMPs),
      mRnd(time(NULL)), mpCl(classifier), mKNN(kNN)
  {
    mpPtamReloc=new BriefRansacRelocaliser<RANSAC,Mod,Desc>(
        model,mpCl,ransacParams,width, height);
//    mRes.retrieved.resize(queryMPs.size(),0);

//    for(uint32_t i=0; i<mQueryMPs.size(); ++i)
//    {
//      assert(mQueryMPs[i]!=NULL);
//      assert(&(mQueryMPs[i]->pt)!=NULL);
//
//      TrackerData* tData=new TrackerData(&(mQueryMPs[i]->pt));
//      mQueryMPs[i]->pt.pTData=tData;
//      tData->nSearchLevel=0; // dont know...
//      tData->dSqrtInvNoise=1/(1<<tData->nSearchLevel);
//      tData->bFound=true;
//      tData->v2Found=mQueryMPs[i]->v2p;
//    }
  };
  ~PtamReLoc ()
  {
//    for(uint32_t i=0; i<mQueryMPs.size(); ++i) delete mQueryMPs[i]->pt.pTData;
    delete mpPtamReloc;
  };

protected:

  void doWork_impl()
  {// work in here
    SE3<double> lastPose=SE3<double>();
    Timer t0;
    vector<Assoc<Desc,uint32_t> > pairing;
    float tPrep=mpCl->prepare();
    float tPair=0.0;
    if(mKNN>1)
      tPair=dynamic_cast<Classifier<Desc,uint32_t>*>(mpCl)->kPair(mQueryMPs,pairing, mKNN);
    else
      tPair=dynamic_cast<Classifier<Desc,uint32_t>*>(mpCl)->pair(mQueryMPs,pairing);
    mpPtamReloc->AttemptRecovery(pairing, lastPose, 30.0);
    t0.toc();
    cout<<"AttemptRecovery time taken: "<<t0<<endl;

    // save results and statistics
    boost::mutex::scoped_lock l(mResMutex); // lock before assigning values

//    const vector<Assoc<Desc,uint32_t> >& pairing=mpPtamReloc->getPairing();
    mRes.p3d.reserve(pairing.size());
    for(uint32_t i=0; i<pairing.size(); ++i){
      if(pairing[i].m != NULL)
        mRes.p3d.push_back(pairing[i].m->pt.v3WorldPos);
    }
    mRes.wTc= mpPtamReloc->BestPose().inverse();
    mRes.inliers=mpPtamReloc->getInlierCount();
    mRes.querySize=mQueryMPs.size();//mQueryMPs.size();
    mRes.mapSize=dynamic_cast<Classifier<Desc,uint32_t>*>(mpCl)->getMapDs().size();
    mRes.tPrep=tPrep;
    mRes.tPair=tPair;
    mRes.tReloc=t0.lastDt();
    cout<<"PtamReLoc Done"<<endl;
  };

private:

  vector<Desc*> mQueryMPs;
  BriefRansacRelocaliser<RANSAC,Mod,Desc>* mpPtamReloc;
  Random mRnd;
  LSH<Desc,uint32_t>* mpCl;
  uint32_t mKNN;
//  Mod mModel;
//  Ransac<Mod> mRansac;
};

template <class Mod, class Desc>
class PtamReLocMEst : public SingleThread<LocResult>
{
public:
  PtamReLocMEst( Mod* model,
      LSH<Desc,uint32_t>* classifier,
      const SE3<double>& lastPose,
      const vector<Desc*>& queryMPs,         // make copy
      double width=640, double height= 480, uint32_t kNN=1)
  :   SingleThread<LocResult>(),
      mQueryMPs(queryMPs),
      mRnd(time(NULL)), mpCl(classifier),
      mLastPose(lastPose),
      mKNN(kNN)
  {
    mpPtamRelocMEst=new BriefMEstRelocaliser<Mod,Desc>(
        model,mpCl,width, height);
//    mRes.retrieved.resize(queryMPs.size(),0);

//    for(uint32_t i=0; i<mQueryMPs.size(); ++i)
//    {
//      assert(mQueryMPs[i]!=NULL);
//      assert(&(mQueryMPs[i]->pt)!=NULL);
//
//      TrackerData* tData=new TrackerData(&(mQueryMPs[i]->pt));
//      mQueryMPs[i]->pt.pTData=tData;
//      tData->nSearchLevel=0; // dont know...
//      tData->dSqrtInvNoise=1/(1<<tData->nSearchLevel);
//      tData->bFound=true;
//      tData->v2Found=mQueryMPs[i]->v2p;
//    }
  };
  ~PtamReLocMEst ()
  {
//    for(uint32_t i=0; i<mQueryMPs.size(); ++i) delete mQueryMPs[i]->pt.pTData;
    delete mpPtamRelocMEst;
  };

protected:

  void doWork_impl()
  {// work in here
    Timer t0;
    vector<Assoc<Desc,uint32_t> > pairing;
    float tPrep=mpCl->prepare();
    float tPair=0.0;
    if(mKNN>1)
      tPair=dynamic_cast<Classifier<Desc,uint32_t>*>(mpCl)->pair(mQueryMPs,pairing,mKNN);
    else
      tPair=dynamic_cast<Classifier<Desc,uint32_t>*>(mpCl)->pair(mQueryMPs,pairing);
    mpPtamRelocMEst->AttemptRecovery(pairing, mLastPose, 30.0);
    t0.toc();
    cout<<"AttemptRecovery time taken: "<<t0<<endl;

    // save results and statistics
    boost::mutex::scoped_lock l(mResMutex); // lock before assigning values

//    const vector<Assoc<Desc,uint32_t> >& pairing=mpPtamReloc->getPairing();
    mRes.p3d.reserve(pairing.size());
    for(uint32_t i=0; i<pairing.size(); ++i){
      if(pairing[i].m != NULL)
        mRes.p3d.push_back(pairing[i].m->pt.v3WorldPos);
    }
    mRes.wTc= mpPtamRelocMEst->BestPose().inverse();
    mRes.inliers=mpPtamRelocMEst->getInlierCount();
    mRes.querySize=mQueryMPs.size();//mQueryMPs.size();
    mRes.mapSize=dynamic_cast<Classifier<Desc,uint32_t>*>(mpCl)->getMapDs().size();
    mRes.tPrep=tPrep;
    mRes.tPair=tPair;
    mRes.tReloc=t0.lastDt();
    cout<<"PtamReLoc Done"<<endl;

  };

private:

  vector<Desc*> mQueryMPs;
  BriefMEstRelocaliser<Mod,Desc>* mpPtamRelocMEst;
  Random mRnd;
  LSH<Desc,uint32_t>* mpCl;
  SE3<double> mLastPose;
  uint32_t mKNN;
//  Mod mModel;
//  Ransac<Mod> mRansac;
};

//template<typename Desc>
//class BriefIRLoc : public BriefIR<Desc>
//{
//public:
//  BriefIRLoc(FeatureDatabase& fdb, LshKBM& lshKBM, ImageSet* imgSet, const IRConfig& config)
//  : BriefIR(fdb, lshKBM, imgSet, config)
//  {};
//  ~BriefIRLoc()
//  {};
//
//  vector<LocResult> search(vector<Desc*>& querMPs,
//      uint32_t frameID, uint32_t kBest)
//      {
//    vector<IrScore> scores;
//    getImgScores(queryMPs,scores,kBest);
//
//    vector<MapPoint*> mapMPs;
//    for(uint32_t i=0; i<kBest; ++i)
//    {
//      const meta_file_entry* pMeta=mFdb.firstMetaEntry(scores[i].imgId);
//      uint8_t* desc = ((uint8_t*)mFdb.firstDescriptor(i)); // for each feature
//      // iterate over the features found in image with index i
//      for(int j=0; j<mFdb.index(scores[i].imgId).desc_count; ++j)
//      {
//        Vector<3,double> pos;
//        pos[0]=pMeta[j].world_x; pos[1]=pMeta[j].world_y; pos[2]=pMeta[j].world_z;
//        MapPoint* mp= new MapPoint();
//        // dont copy data just set pointer to the data
//        BriefData* bD = new BriefData(mp,&desc[j*BriefData::BRIEF_K],false);
//        bD->v2p[0]=pMeta[j].x; bD->v2p[1]=pMeta[j].y;
//        mp->v3WorldPos=pos;
//        mapMPs.push_back(mp);
//      }
//    }
//    vector<SingleThread<LocResult>* > threads;
//    vector<uint32_t> frameIDs;
//    for(uint32_t j=0; j<10; ++j)
//    {
//      threads.push_back(new PtamReLoc<LSH<Desc,uint32_t>,ATANCamera,FourPoint<ATANCamera> >
//      (mConfig.pCam, mapMPs, queryMPs,mConfig.inlierThr,mConfig.ransacTrials,mConfig.consensusThr));
//      frameIDs.push_back(frameID);
//    }
//    MultiThreads<LocResult> multiT(threads,mConfig.nThreadsMax);
//    multiT.work();
//    vector<LocResult> locRes=multiT.getResults();
//
//    cout<<" ----- Results -----"<<endl;
//    for(uint32_t i=0;i<locRes.size(); ++i){
//      locRes[i].dump();
//    }
//
//    for(size_t i=0; i<mapMPs.size(); ++i)
//      locRes[0].p3d.push_back(mapMPs[i]->v3WorldPos);
//
//    threads.clear(); frameIDs.clear();
//    for(size_t i=0; i<mapMPs.size(); ++i)
//    {
//      delete mapMPs[i]->pBData;
//      delete mapMPs[i];
//    }
//
//    return locRes;
//  };
//
//
//  void searchMultHyp(vector<LocResult>& locRes,
//      vector<Desc*>& queryMPs, uint32_t frameID, uint32_t kBest)
//  {
//
//    vector<IrScore> scores;
//    getImgScores(queryMPs,scores,kBest);
//
//    vector<vector<MapPoint*> > mapMPs;
//    vector<SingleThread<LocResult>* > threads;
//    vector<uint32_t> frameIDs;
//
//    for(uint32_t i=0; i<kBest; ++i)
//    {
//      mapMPs.push_back(vector<MapPoint*>());
//      const meta_file_entry* pMeta=mFdb.firstMetaEntry(scores[i].imgId);
//      uint8_t* desc = ((uint8_t*)mFdb.firstDescriptor(i)); // for each feature
//      // iterate over the features found in image with index i
//      for(int j=0; j<mFdb.index(scores[i].imgId).desc_count; ++j)
//      {
//        Vector<3,double> pos;
//        pos[0]=pMeta[j].world_x; pos[1]=pMeta[j].world_y; pos[2]=pMeta[j].world_z;
//        MapPoint* mp= new MapPoint();
//        // dont copy data just set pointer to the data
//        BriefData* bD = new BriefData(mp,&desc[j*BriefData::BRIEF_K],false);
//        bD->v2p[0]=pMeta[j].x; bD->v2p[1]=pMeta[j].y;
//        mp->v3WorldPos=pos;
//        mapMPs[i].push_back(mp);
//      }
//
//      threads.push_back(new PtamReLoc<LSH<desc,uint32_t>,ATANCamera,FourPoint<ATANCamera> >
//        (mConfig.pCam, mapMPs[i], queryMPs, mConfig.inlierThr,mConfig.ransacTrials));
//      frameIDs.push_back(frameID);
//    }
//
//    MultiThreads<LocResult> multiT(threads,mConfig.nThreadsMax);
//    multiT.work();
//    locRes=multiT.getResults();
//
//    cout<<" ----- Results -----"<<endl;
//    for(uint32_t i=0;i<locRes.size(); ++i){
//      locRes[i].dump();
//    }
//
//    for(size_t i=0; i<mapMPs.size(); ++i)
//      for(size_t j=0; j<mapMPs[i].size(); ++j)
//        locRes[0].p3d.push_back(mapMPs[i][j]->v3WorldPos);
//
//    threads.clear(); frameIDs.clear();
//    for(size_t i=0; i<mapMPs.size(); ++i)
//      for(size_t j=0; j<mapMPs[i].size(); ++j){
//        delete mapMPs[i][j]->pBData;
//        delete mapMPs[i][j];
//      }
//  };
//
//private:
//
//};



#endif /* PTAMRELOC_HPP_ */
