

#ifndef __THREADED_CLASSIFIER_H
#define __THREADED_CLASSIFIER_H

#include <helpers.hpp>
#include <MapPoint.h>
#include <BriefData.hpp>
#include <BriefAssociation.hpp>
#include <Classifier.hpp>
#include <Ransac.hpp>
#include <Random.hpp>
#include <MultiThreading.hpp>
#include <BriefRelocalizer.hpp>
#include <Map.h>
#include <PatchFinder.h>
#include <results.hpp>
//#include <TrackerData.h>

#include <fstream>
#include <stdio.h>
#include <vector>
#include <time.h>

using namespace std;


// threaded k nearest neighbour. Result is a ordered
// list of BriefAssociations.
class kNNthreaded : public SingleThread<vector<BriefAssociation> >
{
public:
  kNNthreaded(const vector<MapPoint*>& mapMPs,
      MapPoint* queryMP, uint32_t k)
    : SingleThread<vector<BriefAssociation> >(), mNN(mapMPs), mQueryMP(queryMP), mk(k)
  {};

  ~kNNthreaded()
  {};

protected:
  void doWork_impl()
  {
    vector<BriefAssociation> pairing;
    mNN.pair(mQueryMP,pairing,mk);

    boost::mutex::scoped_lock l(mResMutex);
    mRes.clear();
    for(uint32_t i=0; i<mk; ++i) mRes.push_back(pairing[i]);
  };

private:
  NearestNeighbor mNN;
  MapPoint* mQueryMP; // copy of input queryMPs
  uint32_t mk;
};

template<class Cl>
class ClassifierThreaded : public SingleThread<void>
{
public:
  ClassifierThreaded(const vector<MapPoint*>& mapMPs,
      vector<MapPoint*>::const_iterator& queryMPBegin,
      vector<MapPoint*>::const_iterator& queryMPEnd,
      vector<BriefAssociation>::iterator& pairingBegin,
      vector<BriefAssociation>::iterator& pairingEnd,
      uint32_t p1=4, uint32_t p2=5)
  : SingleThread<void>(),
    mP1(p1), mP2(p2), mpCl(new Cl(mapMPs,mP1,mP2)),
    mQueryMPBegin(queryMPBegin), mQueryMPEnd(queryMPEnd),
    mPairingBegin(pairingBegin), mPairingEnd(pairingEnd),
    mClIsManagedByThis(true)
    {};

  ClassifierThreaded(Cl* pLsh,
      vector<MapPoint*>::const_iterator& queryMPBegin,
      vector<MapPoint*>::const_iterator& queryMPEnd,
      vector<BriefAssociation>::iterator& pairingBegin,
      vector<BriefAssociation>::iterator& pairingEnd)
  : SingleThread<void>(),
    mpCl(pLsh),
    mQueryMPBegin(queryMPBegin), mQueryMPEnd(queryMPEnd),
    mPairingBegin(pairingBegin), mPairingEnd(pairingEnd),
    mClIsManagedByThis(false)
    {};

  ~ClassifierThreaded()
  {
    if(mClIsManagedByThis) delete mpCl;
  };

protected:
  void doWork_impl()
  {
    uint32_t nRetrieved=0;
    BriefAssociation pairing;
    if(mClIsManagedByThis) mpCl->prepare();
    if(mQueryMPBegin==mQueryMPEnd)
      mpCl->pair(*mQueryMPBegin,*mPairingBegin);
    else
      mpCl->pair(mQueryMPBegin,mQueryMPEnd,mPairingBegin,mPairingEnd);
  };

private:

  uint32_t mP1;
  uint32_t mP2;
  Cl* mpCl;

  vector<MapPoint*>::const_iterator mQueryMPBegin;
  vector<MapPoint*>::const_iterator mQueryMPEnd;
  vector<BriefAssociation>::iterator mPairingBegin;
  vector<BriefAssociation>::iterator mPairingEnd;

  bool mClIsManagedByThis;
};

// NNthreaded or LSHthreaded
template <class Cl>
class kBinaryMeansThreaded : public kBinaryMeans
{
public:
  kBinaryMeansThreaded(const vector<MapPoint*>& mp, Random& rnd,
      uint32_t maxThreads=3)
    : kBinaryMeans(mp,rnd), mMaxThreads(maxThreads),
//      mNNPacketCount(mMaxThreads*1000), mNNPacketSize(uint32_t(ceil(double(N)/double(mNNPacketCount)))),
      mNNPacketSize(1000), mVotingPacketSize(25)
  {
    mNNPacketCount=uint32_t(ceil(double(N)/double(mNNPacketSize)));
    mNNThreads.resize(mNNPacketCount,NULL);
    mPairing.resize(N);
  };

  ~kBinaryMeansThreaded()
  {};

  // clustering into k clusters -> use the same as the base class
  double cluster(uint32_t k_)
  {
  // --------------------- Init
    k=k_; // how many clusters do we want?
    mCs.resize(k,NULL);
    mClusters.resize(k,vector<MapPoint*>());
    mCentroidsLUT.clear();

    mVotingPacketCount=uint32_t(ceil(double(k)/double(mVotingPacketSize)));
    mVotingThreads.resize(mVotingPacketCount);

    cout<<"kBinaryMeansThreaded::cluster: N="<<N<<" mMp.size="<<mMp.size()<<endl;
    for(uint32_t i=0; i<k; ++i)
    {
      mCs[i]=new MapPoint();
      mCs[i]->pBData=new BriefData(mCs[i]);// initialize centroids randomly from datapoints
      uint32_t nRnd=mRnd.drawWithoutRepetition(N,i==0);
      mCs[i]->pBData->copyDataFrom(mMp[nRnd]->pBData);
      mCentroidsLUT[mCs[i]]=i;
    }
//    for(uint32_t i=0; i<mCs.size(); ++i) cout<<"@i="<<i<<" "<<*mCs[i]->pBData<<endl;
    cout<<"kBinaryMeansThreaded::cluster: Initialized centroids"<<endl;
  // --------------------- EM to find k means
    double J=0.0, Jprev=256.0, eps=1e-5;
    bool converged=false; // did the centroids change?
    uint32_t it=0;
    while(!converged)
    {
      // assign datapoints to the centroids mCs
      cout<<"kBinaryMeansThreaded::cluster: Evaluating cost function"<<endl;
      J=evalCost();
      cout<<"kBinaryMeansThreaded::cluster: (Cost) J="<<J<<" @iteration="<<it<<endl;
//      for(uint32_t i=0; i<mCs.size(); ++i) cout<<"@i="<<i<<" "<<*mCs[i]->pBData<<endl;

      cout<<"kBinaryMeansThreaded::cluster: Recomputing means"<<endl;
      // recompute means; check whether means have changed and then copy current means to csPrev

      vector<vector<MapPoint*> >::const_iterator itCl=mClusters.begin();
      vector<vector<MapPoint*> >::const_iterator itClBegin=itCl;
      vector<MapPoint*>::iterator itCsBd=mCs.begin();
      vector<MapPoint*>::iterator itCsBdBegin=itCsBd;

      for (uint32_t j=0; j<mVotingPacketCount-1; ++j){
        itClBegin=itCl;
        itCsBdBegin=itCsBd;
        advance(itCl,mVotingPacketSize);
        advance(itCsBd,mVotingPacketSize);
        mVotingThreads[j]=new ComputeMeanThreaded(itClBegin,itCl,itCsBdBegin,itCsBd);
//        cout<<"reading "<<distance(itClBegin,itCl-1)<<": "<<
//            distance(vector<vector<MapPoint*> >::const_iterator(mClusters.begin()),itClBegin)<<" to "<<
//            distance(vector<vector<MapPoint*> >::const_iterator(mClusters.begin()),itCl-1)<<" of "<<N<<" packet "<<
//                  j<<" of "<<mVotingPacketCount<<" of size="<<mVotingPacketSize<<endl;
      }
      itClBegin=itCl; itCl=mClusters.end();
      itCsBdBegin=itCsBd; itCsBd=mCs.end();
      mVotingThreads[mVotingPacketCount-1]=new ComputeMeanThreaded(itClBegin,itCl,itCsBdBegin,itCsBd);

      {
        double meanClSize=0.0, minClSize=FLT_MAX, maxClSize=-FLT_MAX;
        for(uint32_t i=0; i<k; ++i){
          meanClSize+=mClusters[i].size();
          if(mClusters[i].size()>maxClSize) maxClSize=mClusters[i].size();
          if(mClusters[i].size()<minClSize) minClSize=mClusters[i].size();
        }
        cout<<"kBinaryMeansThreaded::cluster: cluster size: min="<<minClSize<<" mean="<<meanClSize/double(k)<<" max="<<maxClSize<<endl;
      }

      MultiThreads<void> multiT(mVotingThreads,mMaxThreads);
      multiT.work();

      double dJ=Jprev-J;
      cout<<"kBinaryMeansThreaded::cluster: dJ="<<dJ<<endl;
      converged=((dJ<eps) && (dJ>=0.0));
      Jprev=J;

      ++it;
    }
//    J=evalCost(); // not necessary!
    cout<<"kBinaryMeansThreaded::cluster: final J="<<J<<endl;
    return J; // return score
  };

//  double clusterMultiThreaded(uint32_t k);

protected:

  uint32_t mMaxThreads;

  // evaluate kBinaryMeans costfunction -> multithreaded
  double evalCost()
  {
    Cl* pCl=new Cl(mCs); // l=4, m=5 in case this Cl is LSH; If Cl=NN the parameters do not matter anyway
    float dt=pCl->prepare(); // prepare LSH here for all threads!
    cout<<"kBinaryMeansThreaded::evalCost: preparing Classifier dt="<<dt<<"ms"<<endl;

    cout<<mNNPacketCount<<" Threads with "<<mNNPacketSize<<" mappoints per thread"<<endl;
    vector<MapPoint*>::const_iterator itMp=mMp.begin();
    vector<MapPoint*>::const_iterator itMpBegin=itMp;
    vector<BriefAssociation>::iterator itPair=mPairing.begin();
    vector<BriefAssociation>::iterator itPairBegin=itPair;
    for (uint32_t j=0; j<mNNPacketCount-1; ++j){
      itMpBegin=itMp;
      itPairBegin=itPair;
      advance(itMp,mNNPacketSize);
      advance(itPair,mNNPacketSize);
      mNNThreads[j]=new ClassifierThreaded<Cl>(pCl,itMpBegin,itMp,itPairBegin,itPair);
//      cout<<"reading "<<distance(itMpBegin,itMp-1)<<": "<<distance(mMp.begin(),itMpBegin)<<" to "<<distance(mMp.begin(),itMp-1)<<" of "<<N<<" packet "<<
//          j<<" of "<<mNNPacketCount<<" of size="<<mNNPacketSize<<" :"<<*itMpBegin<<endl;
    };
    itMpBegin=itMp; itMp=mMp.end();
    itPairBegin=itPair; itPair=mPairing.end();
    mNNThreads[mNNPacketCount-1]=new ClassifierThreaded<Cl>(pCl,itMpBegin,itMp,itPairBegin,itPair);
//    cout<<"reading "<<distance(itMpBegin,itMp-1)<<": "<<distance(mMp.begin(),itMpBegin)<<" to "<<distance(mMp.begin(),itMp-1)<<" of "<<N<<" packet "<<
//        mNNPacketCount-1<<" of "<<mNNPacketCount<<" of size="<<mNNPacketSize<<" :"<<*itMpBegin<<endl;


    MultiThreads<void> multiT(mNNThreads,mMaxThreads);
    cout<<"Running "<<mMaxThreads<<" threads for NN evaluation"<<endl;
    multiT.work();

    cout<<"Filling clusters with NNs"<<endl;
    for (uint32_t i=0; i<k; ++i) mClusters[i].clear();
    double J=0.0;
    for (uint32_t j=0; j<N; ++j){
      if(mPairing[j].dist>256) continue; // simply discard unmatched features - do not add them to clusters;
//      cout<<"MapPointNr="<<j<<" dist="<<mPairing[j].dist<<" mp1="<<mPairing[j].mp1<<" mp2="<<mPairing[j].mp2<<endl;
      // unmatched features originate from aproximate NN
      mClusters[mCentroidsLUT[mPairing[j].mp2]].push_back(mMp[j]);
      J+=mPairing[j].dist;
      ASSERT(mPairing[j].dist<257, "MapPointNr="<<j<<" dist="<<mPairing[j].dist<<" mp1="<<mPairing[j].mp1<<" mp2="<<mPairing[j].mp2);
    }

    delete pCl;
    return J/double(N);
  };

private:

  uint32_t mNNPacketCount, mNNPacketSize;
  uint32_t mVotingPacketCount, mVotingPacketSize;

  vector<SingleThread<void>* > mNNThreads;
  vector<SingleThread<void>* > mVotingThreads;
  vector<BriefAssociation> mPairing;

//  double evalCostMultiThreaded(const vector<MapPoint*>& cs);
};

template<class KBM>
class kBinMeansMulti
{ // uses RAII
public:
  kBinMeansMulti(const vector<MapPoint*>& mapMPs, uint32_t k,
      string outputPath) :
    mk(k), mMapMPs(mapMPs),
    mKBinMeans(mapMPs, time(NULL), mk),
    mTPrep(0.0), mOutputPath(outputPath),
    mStopRequested(false), mIsRunning(true),
    mThread(boost::bind(&kBinMeansMulti::doWork, this))
  {};
  ~kBinMeansMulti()
  {
    mStopRequested = true;
    mThread.join();
  };

  bool isRunning()
  {
    return mIsRunning;
  }

  double getTPrep()
  {
    boost::mutex::scoped_lock l(mResMutex); // lock before retrieving values
    return mTPrep;
  }

  KBM& getKBinMeans(void)
  {
    return mKBinMeans;
  };

private:

  uint32_t mk;
  const vector<MapPoint*>& mMapMPs;
  KBM mKBinMeans;
  double mTPrep; // time for clustering

  string mOutputPath;

  volatile bool mStopRequested;
  volatile bool mIsRunning;
  boost::thread mThread;
  boost::mutex mResMutex;

  void doWork()
  {
    while(!mStopRequested)
    {// work in here
      double tPrep=0.0;
      char buf[100]; sprintf(buf,"%s%04dMeans.txt",mOutputPath.c_str(),mk);
      ifstream in(buf);
      if(in)
      {// if the file already exists (ie we already computed centroids) avoid recomputing
        mKBinMeans.getClusterer().loadCentroidsFromFile(buf);
        tPrep=dynamic_cast<Classifier*>(&mKBinMeans)->prepare(mk); // to prepare the LSH (if we are using it)
      }else{ // compute centroids
        tPrep=dynamic_cast<Classifier*>(&mKBinMeans)->prepare(mk); // clustering
        cout<<"#Centroids="<<mKBinMeans.getClusterer().getCentroids().size()<<endl;
        mKBinMeans.getClusterer().saveCentroidsToFile(buf); //save for next time
        // save preparation time (clustering time)
        sprintf(buf,"%skBM%05dMeansTimePrep.txt",mOutputPath.c_str(),mk);
        ofstream outTimePrep(buf);
        outTimePrep<<tPrep<<endl;
        outTimePrep.close();
        // save cluster sizes
        sprintf(buf,"%skBM%05dMeansClusterSize.txt",mOutputPath.c_str(),mk);
        ofstream outClusterSize(buf);
        vector<MapPoint*> cs= mKBinMeans.getClusterer().getCentroids();
        for (uint32_t i=0; i<cs.size(); ++i)
          outClusterSize<<mKBinMeans.getClusterer().getCluster(cs[i]).size()<<endl;
        outClusterSize.close();
      }
      boost::mutex::scoped_lock l(mResMutex); // lock before assigning values
      mTPrep=tPrep;
      mStopRequested=true;
    };
    mIsRunning=false;
  };
};

// LSHthreaded for evaluation purposes
class LSHthreadedEval : public SingleThread<ClassifierResult>
{
public:
  LSHthreadedEval(const vector<MapPoint*>& mapMPs, vector<MapPoint*>& queryMPs,
      const vector<BriefAssociation>& pairingNN, uint32_t m, uint32_t l)
  : SingleThread<ClassifierResult>(),
      mm(m), ml(l), mMapMPs(mapMPs), mQueryMPs(queryMPs),
      mPairingNN(pairingNN),
      mLSH(mapMPs, l, m)
  {

  };
  ~LSHthreadedEval()
  {};

  void getParams(uint32_t& m, uint32_t& l) {m=mm; l=ml;};

private:

  uint32_t mm,ml;
  const vector<MapPoint*>& mMapMPs;
  vector<MapPoint*>& mQueryMPs;
  const vector<BriefAssociation>& mPairingNN; // groundtruth pairing
  LSH mLSH;

  void doWork_impl()
  {// work in here
    vector<BriefAssociation> pairingLSH;
    float tPrep=mLSH.prepare();
    vector<uint32_t> retrieved;
    float tPair=mLSH.pair(mQueryMPs,pairingLSH,retrieved);
    // print some stuff
    //        cout<<"--------- Time taken by LSH: "<<tPair<<"ms"<<endl;
    //        printPairing(briefFrames,pairingLSH);
    // compute statistics
    boost::mutex::scoped_lock l(mResMutex); // lock before assigning values
    mRes.tPrep=tPrep;
    mRes.tPair=tPair;
    mRes.retrieved=retrieved;
    evalPairing(pairingLSH, mRes.avgDist, mRes.paired);
    compairAgainstNN(pairingLSH, mPairingNN, mRes.tp);
    cout<<"LSH tread l="<<ml<<" m="<<mm<<" stopped #tp="<<mRes.tp<<endl;
  };
};

#endif
