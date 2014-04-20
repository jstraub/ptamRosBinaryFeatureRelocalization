/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef KBM_HPP_
#define KBM_HPP_

#include <MultiThreading.hpp>
#include <timer.hpp>
#include <Random.hpp>

#include <time.h>
#include <vector>
#include <map>

using namespace std;

template<typename Desc>
class ComputeMeanThreaded : public SingleThread<void>
{
public:
  ComputeMeanThreaded(
      typename vector<vector<Desc*> >::const_iterator& clustersBegin,
      typename vector<vector<Desc*> >::const_iterator& clustersEnd,
      typename vector<Desc*>::iterator& centroidBegin,
      typename vector<Desc*>::iterator& centroidEnd)
  : SingleThread<void>(),
    mClustersBegin(clustersBegin), mClustersEnd(clustersEnd),
    mCentroidBegin(centroidBegin), mCentroidEnd(centroidEnd)
  {};
  ~ComputeMeanThreaded()
  {};

protected:
  void doWork_impl()
  {
    vector<uint32_t> score(Desc::BRIEF_K*8,0);
    typename vector<Desc*>::iterator itCentroid=mCentroidBegin;
    uint32_t nCentroid=0;
    for(typename vector<vector<Desc*> >::const_iterator itCl=mClustersBegin;
        itCl !=mClustersEnd; itCl++, itCentroid++, nCentroid++)
    {
      assert(itCl!=mClustersEnd); assert(itCentroid!=mCentroidEnd);
      for(uint32_t j=0; j<(*itCl).size(); ++j)
      {// basically voting on bit positions in the BriefDescriptor form all descriptors in the cluster
//        cout<<"j="<<j<<" "<<(*itCl)[j]<<" "<<(*itCentroid)<<endl;
        (*itCl)[j]->incrBriefScore(score);
      }
      // obtain new cluster center from the voting on the bit positions -> new mean in the binary vectorspace
      (*itCentroid)->descriptorFromBriefScore(score, (*itCl).size());
      for(uint32_t i=0; i<score.size();++i) score[i]=0;
//      cout<<"Updated Centroid "<<nCentroid<<" of "<<distance(mCentroidBegin,mCentroidEnd)<<endl;
    }
//    cout<<"Done with ComputeMeanThreaded"<<endl;
  };
private:

  typename vector<vector<Desc*> >::const_iterator mClustersBegin;
  typename vector<vector<Desc*> >::const_iterator mClustersEnd;
  typename vector<Desc*>::iterator mCentroidBegin;
  typename vector<Desc*>::iterator mCentroidEnd;

};

struct ClusterSize
{
  ClusterSize(uint32_t size_=0, uint32_t id_=0) : size(size_), id(id_)
  {};
  uint32_t size;
  uint32_t id;
  bool operator<(const ClusterSize& clS) const
  {
    return size<clS.size;
  }
};

template<class Desc, class Dist, class Cl>
class kBinaryMeansThreaded
{
public:
  kBinaryMeansThreaded(const vector<Desc*>& briefDs, Params* pClParams, uint32_t maxThreads=3)
  : mBriefDs(briefDs), mRnd(time(NULL)), mpClParams(pClParams), mMaxThreads(maxThreads),
    mNNPacketSize(1000), mVotingPacketSize(25), mReassignPacketSize(20)
  {
    mNNPacketCount=uint32_t(ceil(double(briefDs.size())/double(mNNPacketSize)));
    mNNThreads.resize(mNNPacketCount,NULL);
    mPairing.resize(briefDs.size());
  };
  ~kBinaryMeansThreaded()
  {};


//  const vector<Desc*>* getCentroids(){
//      return &mCs;
//    };
  const vector<Desc*>& getCentroids(){
    return mCs;
  };
  void getCentroids(vector<Desc*>& centroids){
    centroids.resize(mCs.size(),NULL);
    for(uint32_t i=0; i<mCs.size(); ++i)
      centroids[i]=mCs[i];
  };
  vector<Desc*>& getCluster(uint32_t id) {
    return mClusters[id];
  };
  vector<Desc*>& getCluster(Desc* centroid) {
    return mClusters[mCentroidsLUT[centroid]];
  };

  double cluster(uint32_t k_, bool doBalanceNullclusters=true)
  {
  // --------------------- Init
    k=k_; // how many  do we want?
    uint32_t meanClusterSize=ceil(double(mBriefDs.size())/double(k));
    mCs.resize(k,NULL);
    mClusters.resize(k,vector<Desc*>());
    mClSizes.resize(k,0);
    mCentroidsLUT.clear();

    mVotingPacketCount=uint32_t(ceil(double(k)/double(mVotingPacketSize)));
    mVotingThreads.resize(mVotingPacketCount);

    cout<<"kBinaryMeansThreaded::cluster: N="<<mBriefDs.size()<<endl;
    for(uint32_t i=0; i<k; ++i)
    {
      uint32_t nRnd=mRnd.drawWithoutRepetition(mBriefDs.size(),i==0);
      mCs[i]=new BriefDescS(mBriefDs[nRnd]);// initialize centroids randomly from datapoints
      mCentroidsLUT[mCs[i]]=i;
    }
//    for(uint32_t i=0; i<mCs.size(); ++i) cout<<"@i="<<i<<" "<<*mCs[i]->pBData<<endl;
    cout<<"kBinaryMeansThreaded::cluster: Initialized "<<k<<" centroids"<<endl;
    double J=evalCost();
    cout<<"kBinaryMeansThreaded::cluster: Initialized "<<k<<" clusters. Initial \e[0;31mJ="<<J<<"\e[m"<<endl;
  // --------------------- EM to find k means
    double Jprev=J, eps=1e-5;
    bool converged=false; // did the centroids change?
    bool finishOnNextDecrease=false;
    uint32_t it=0;
    while(!converged)
    {
      // handle empty clusters
      vector<Desc*> nullClusters;
      checkForNullClusters(nullClusters);
      if(doBalanceNullclusters)
      {
        while(nullClusters.size()>0)
        {
          sort(mClSizes.begin(),mClSizes.end());
          vector<Desc*> clusterDs;
          //copy enough datapoints from the biggest clusters to allow sampling new centroids from them.
          uint32_t o=k-1;
          double dJ=0;
          // collect as many feature in clusterDs as necessary to obtain clusters with
          // the mean cluster size
          while(clusterDs.size()<min((nullClusters.size()+(k-1-o))*(meanClusterSize-1),mBriefDs.size()))
          { // need more datapoints
            //                      cout<<"(@"<<mClSizes[o].id<<","<<o<<")"<<mClSizes[o].size<<" ";
            clusterDs.reserve(clusterDs.size() + mClSizes[o].size);
            assert(mClSizes[o].size == mClusters[mClSizes[o].id].size());
            for(uint32_t l=0; l<mClSizes[o].size; ++l)
            { // copy them
              clusterDs.push_back(mClusters[mClSizes[o].id][l]);
              // subtract the distances from J since we are going to reassign and recompute them
              dJ += mCs[mClSizes[o].id]->dist(mClusters[mClSizes[o].id][l]);
            }
            mClusters[mClSizes[o].id].clear(); // and clear the cluster
            --o;
          }; //cout<<endl;
          J-=dJ/double(mBriefDs.size());
          //        cout<<"J-="<<dJ/double(mBriefDs.size())<<endl;

          vector<Desc*> nullClusterDs; nullClusterDs.reserve(nullClusters.size());
          for(uint32_t i=0; i<nullClusters.size(); ++i)
          {
            // get all features that are in the empty clusters (empty means 0 or 1 feature)
            uint32_t clId=mCentroidsLUT[nullClusters[i]];
            for(uint32_t j=0; j<mClusters[clId].size(); ++j)
              nullClusterDs.push_back(mClusters[clId][j]);
            // draw new centroids
            nullClusters[i]->copyDataFrom(clusterDs[mRnd.drawWithoutRepetition(clusterDs.size(),i==0)]);
          }
          for(uint32_t i=k-1; i>o; --i)
          { // append cluster centers of the cluster which we are taking the points form.
            nullClusters.push_back(mCs[mClSizes[i].id]);
            // and reinitialize them randomly as well?!
            //          nullClusters[nullClusters.size()-1]->copyDataFrom(
            //              clusterDs[mRnd.drawWithoutRepetition(clusterDs.size())]);
          }
          cout<<"kBinaryMeansThreaded::reassign: reassigning "<<nullClusters.size()<<" centroids from the "
              <<(k-1-o)<<" largest clusters with a total of "<<clusterDs.size()<<" points"<<endl;
          // do the reassigning of the centroids which were drawn randomly and of the centroids
          // which belonged to the largest clusters
          // also add the J of all reassigned points to dJ

          // TODO: shouldnt I do a recomputation of the centroids as well
          // DONE: where do I update mCs??? -> nullClusters is only pointers to centroids in mCs
          J += reassign(nullClusters, clusterDs);
          // reassign features from the empty clusters to any of all clusters
          if(nullClusterDs.size()>0) J += reassign(mCs,nullClusterDs);
          nullClusters.clear();
          checkForNullClusters(nullClusters);
        }
      }else{
        cout<<"No empty cluster - overfull cluster balancing."<<endl;
      }
      cout<<"kBinaryMeansThreaded::cluster: Recomputing means"<<endl;
      // recompute means; check whether means have changed and then copy current means to csPrev
      recomputeMeans();
      // assign datapoints to the centroids mCs
      cout<<"kBinaryMeansThreaded::cluster: Evaluating cost function"<<endl;
      J=evalCost();
      double dJ=Jprev-J;
      Jprev=J;
      cout<<"kBinaryMeansThreaded::cluster: \e[0;31m J="<<J<<" dJ="<<dJ<<" @iteration="<<it<<"\e[m"<<endl;
      //      if(!finishOnNextDecrease) finishOnNextDecrease=(dJ<0); // negative steps -> from now on J will not change much anymore -> stop once we have decreased the next time.
      converged=(((dJ<eps) && (dJ>=0.0))||(finishOnNextDecrease && (dJ>0))||(it>=30));

      ++it;
    }
    //    J=evalCost(); // not necessary!
    cout<<"kBinaryMeansThreaded::cluster: final J="<<J<<endl;
    return J; // return score
  };

private:

  const vector<Desc*>& mBriefDs; // datapoints
  vector<Desc*> mCs; // current clustering centroids
  vector<vector<Desc*> > mClusters;
  map<Desc*,uint32_t> mCentroidsLUT; // Lookup table to find centroid ID quickly
  Random mRnd; // random number generator for clustering
  uint32_t k; // number of clusters

  Params* mpClParams;

  uint32_t mMaxThreads;
  uint32_t mNNPacketCount, mNNPacketSize;
  uint32_t mVotingPacketCount, mVotingPacketSize;
  uint32_t mReassignPacketSize;

  vector<SingleThread<void>* > mNNThreads;
  vector<SingleThread<void>* > mVotingThreads;
  vector<Assoc<Desc,Dist> > mPairing;

  vector<ClusterSize> mClSizes; // sizes of each cluster

  void recomputeMeans()
  {
    typename vector<vector<Desc*> >::const_iterator itCl=mClusters.begin();
    typename vector<vector<Desc*> >::const_iterator itClBegin=itCl;
    typename vector<Desc*>::iterator itCsBd=mCs.begin();
    typename vector<Desc*>::iterator itCsBdBegin=itCsBd;

    for (uint32_t j=0; j<mVotingPacketCount-1; ++j){
      itClBegin=itCl;
      itCsBdBegin=itCsBd;
      advance(itCl,mVotingPacketSize);
      advance(itCsBd,mVotingPacketSize);
      mVotingThreads[j]=new ComputeMeanThreaded<Desc>(itClBegin,itCl,itCsBdBegin,itCsBd);
      //        cout<<"reading "<<distance(itClBegin,itCl-1)<<": "<<
      //            distance(vector<vector<MapPoint*> >::const_iterator(mClusters.begin()),itClBegin)<<" to "<<
      //            distance(vector<vector<MapPoint*> >::const_iterator(mClusters.begin()),itCl-1)<<" of "<<N<<" packet "<<
      //                  j<<" of "<<mVotingPacketCount<<" of size="<<mVotingPacketSize<<endl;
    }
    itClBegin=itCl; itCl=mClusters.end();
    itCsBdBegin=itCsBd; itCsBd=mCs.end();
    mVotingThreads[mVotingPacketCount-1]=new ComputeMeanThreaded<Desc>(
        itClBegin,itCl,itCsBdBegin,itCsBd);

    MultiThreads<void> multiT(mVotingThreads,mMaxThreads);
    multiT.work();
  };

  // evaluate kBinaryMeans costfunction
  double evalCost()
  {
    Cl* pCl=new Cl(mCs,*mpClParams); // l=4, m=5 in case this Cl is LSH; If Cl=NN the parameters do not matter anyway
    float dt=pCl->prepare(); // prepare LSH here for all threads!
    cout<<"kBinaryMeansThreaded::evalCost: preparing Classifier dt="<<dt<<"ms"<<endl;

    cout<<"  "<<mNNPacketCount<<" Threads with "<<mNNPacketSize<<" mappoints per thread"<<endl;
    typename vector<Desc*>::const_iterator itMp=mBriefDs.begin();
    typename vector<Desc*>::const_iterator itMpBegin=itMp;
    typename vector<Assoc<Desc,Dist> >::iterator itPair=mPairing.begin();
    typename vector<Assoc<Desc,Dist> >::iterator itPairBegin=itPair;
    for (uint32_t j=0; j<mNNPacketCount-1; ++j){
      itMpBegin=itMp;
      itPairBegin=itPair;
      advance(itMp,mNNPacketSize);
      advance(itPair,mNNPacketSize);
      mNNThreads[j]=new ClassifierThreaded<Desc,Dist>(pCl,itMpBegin,itMp,itPairBegin,itPair);
//      cout<<"reading "<<distance(itMpBegin,itMp-1)<<": "<<distance(mMp.begin(),itMpBegin)<<" to "<<distance(mMp.begin(),itMp-1)<<" of "<<N<<" packet "<<
//          j<<" of "<<mNNPacketCount<<" of size="<<mNNPacketSize<<" :"<<*itMpBegin<<endl;
    };
    itMpBegin=itMp; itMp=mBriefDs.end();
    itPairBegin=itPair; itPair=mPairing.end();
    mNNThreads[mNNPacketCount-1]=new ClassifierThreaded<Desc,Dist>(pCl,itMpBegin,itMp,itPairBegin,itPair);
//    cout<<"reading "<<distance(itMpBegin,itMp-1)<<": "<<distance(mMp.begin(),itMpBegin)<<" to "<<distance(mMp.begin(),itMp-1)<<" of "<<N<<" packet "<<
//        mNNPacketCount-1<<" of "<<mNNPacketCount<<" of size="<<mNNPacketSize<<" :"<<*itMpBegin<<endl;

    MultiThreads<void> multiT(mNNThreads,mMaxThreads);
    cout<<"  Running "<<mMaxThreads<<" threads for NN evaluation"<<endl;
    multiT.work();

    cout<<"  Filling clusters with NNs"<<endl;
    for (uint32_t i=0; i<k; ++i) mClusters[i].clear();
    double J=0.0;
    for (uint32_t j=0; j<mBriefDs.size(); ++j){
      if(mPairing[j].d>256) continue; // simply discard unmatched features - do not add them to clusters;
//      cout<<"MapPointNr="<<j<<" dist="<<mPairing[j].dist<<" mp1="<<mPairing[j].mp1<<" mp2="<<mPairing[j].mp2<<endl;
      // unmatched features originate from aproximate NN
      mClusters[mCentroidsLUT[mPairing[j].m]].push_back(mBriefDs[j]);
      J+=mPairing[j].d;
      assert(mPairing[j].d<257);
//      ASSERT(mPairing[j].dist<257, "MapPointNr="<<j<<" dist="<<mPairing[j].dist<<" mp1="<<mPairing[j].mp1<<" mp2="<<mPairing[j].mp2);
    }

    delete pCl;
    return J/double(mBriefDs.size());
  };

  double reassign(vector<Desc*>& centroids, vector<Desc*>& clusterDs)
  {
//    cout<<"clusterDs:"<<endl;
//    for (uint32_t j=0; j<clusterDs.size(); ++j){
//          cout<<clusterDs[j]<< " "<<*clusterDs[j]<<endl;
//        }
//    cout<<"nullClusters:"<<endl;
//    for (uint32_t j=0; j<nullClusters.size(); ++j){
//          cout<<nullClusters[j]<< " "<<*nullClusters[j]<<endl;
//        }

    cout<<"clusterDs.size()="<<clusterDs.size()<<endl;
    assert(clusterDs.size()>0);

    Cl* pCl=new Cl(centroids,*mpClParams);
    float dt=pCl->prepare(); // prepare LSH here for all threads!
    cout<<"  preparing Classifier dt="<<dt<<"ms"<<endl;

    uint32_t nThreads= uint32_t(ceil(double(clusterDs.size())/double(mReassignPacketSize)));
    cout<<"  "<<(nThreads-1)<<" Threads with "<<mReassignPacketSize<<" mappoints per thread";

    vector<Assoc<Desc,Dist> > pairing(clusterDs.size(),Assoc<Desc,Dist>());
    typename vector<Desc*>::const_iterator itMp=clusterDs.begin();
    typename vector<Desc*>::const_iterator itMpBegin=itMp;
    typename vector<Assoc<Desc,Dist> >::iterator itPair=pairing.begin();
    typename vector<Assoc<Desc,Dist> >::iterator itPairBegin=itPair;
    vector<SingleThread<void>* > nnThreads(nThreads,NULL);
//    cout<<"nnThreads.size()="<<nnThreads.size()<<" nThreads="<<nThreads<<endl;

    assert(nThreads>0);
    for (uint32_t j=0; j<nThreads-1; ++j){
      itMpBegin=itMp;
      itPairBegin=itPair;
      advance(itMp,mReassignPacketSize);
      advance(itPair,mReassignPacketSize);
      assert(itMp!=clusterDs.end()); assert(itPair!=pairing.end());
      nnThreads[j]=new ClassifierThreaded<Desc,Dist>(pCl,itMpBegin,itMp,itPairBegin,itPair);
    };
    itMpBegin=itMp; itMp=clusterDs.end();
    itPairBegin=itPair; itPair=pairing.end();
    nnThreads[nThreads-1]=new ClassifierThreaded<Desc,Dist>(pCl,itMpBegin,itMp,itPairBegin,itPair);
    cout<<" and one thread with "<<distance(itMpBegin,itMp)<<" mappoints"<<endl;

//    for (uint32_t j=0; j<nThreads; ++j){
//      cout<<nnThreads[j]<<" ";
//    }; cout<<endl;

    MultiThreads<void> multiT(nnThreads,mMaxThreads);
    cout<<"  Running "<<mMaxThreads<<" threads for NN evaluation"<<endl;
    multiT.work();

    double J_reassign=0.0;
    for (uint32_t j=0; j<pairing.size(); ++j){
      if(pairing[j].d>256) continue; // simply discard unmatched features - do not add them to clusters;
      // unmatched features originate from aproximate NN
      mClusters[mCentroidsLUT[pairing[j].m]].push_back(clusterDs[j]);
      assert(pairing[j].d<257);
      J_reassign+=pairing[j].d;
    }
    delete pCl;

    // check statistics after reassigning.
//    uint32_t meanClSize=0.0, minClSize=INT_MAX, maxClSize=0, nNullCl=0;
//    for(uint32_t i=0; i<k; ++i){
//      meanClSize+=mClusters[i].size();
//      if(mClusters[i].size()>maxClSize){maxClSize=mClusters[i].size();}
//      if(mClusters[i].size()<minClSize) minClSize=mClusters[i].size();
//      if(mClusters[i].size()<2) ++nNullCl;
//    }
//    cout<<"kBinaryMeansThreaded::cluster: after reassigning: cluster size:"
//        <<" min="<<minClSize<<" ("<<nNullCl<<" nullClusters)"
//        <<" mean="<<meanClSize/double(k)
//        <<" max="<<maxClSize<<endl;

//    cout<<" J += " <<   J_reassign/double(mBriefDs.size())<<endl;
    return J_reassign/double(mBriefDs.size());
  }

  void checkForNullClusters(vector<Desc*>& nullClusters)
  {
    uint32_t meanClSize=0.0, minClSize=INT_MAX, maxClSize=0;
    for(uint32_t i=0; i<k; ++i){
      meanClSize+=mClusters[i].size();
      mClSizes[i]=ClusterSize(mClusters[i].size(),i);
      if(mClusters[i].size()>maxClSize){ maxClSize=mClusters[i].size();}
      if(mClusters[i].size()<minClSize) minClSize=mClusters[i].size();
      if(mClusters[i].size()<2){
        nullClusters.push_back(mCs[i]);
      }
    }
    cout<<"kBinaryMeansThreaded::cluster: cluster size:"
        <<" min="<<minClSize<<" ("<<nullClusters.size()<<" nullClusters)"
        <<" mean="<<meanClSize/double(k)
        <<" max="<<maxClSize<<endl;
  }

  // declare private and do not implement -> no copy constructor
  kBinaryMeansThreaded& operator=(const kBinaryMeansThreaded&);
};



#endif /* KBM_HPP_ */
