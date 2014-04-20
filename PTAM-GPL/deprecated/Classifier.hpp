/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 
#ifndef CLASSIFIER_HPP_
#define CLASSIFIER_HPP_

#include "MapPoint.h"
#include "BriefData.hpp"
#include "BriefRelocalizer.hpp"
#include "BriefAssociation.hpp"
#include "Random.hpp"
#include "timer.hpp"
#include "Assert.h"

#include <boost/foreach.hpp>

#include <vector>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <algorithm>

using namespace std;

struct BriefAssociation; // otherwise we get a circular inclusion


class kCluster
{
public:

  kCluster(const vector<MapPoint*>& mp, Random& rnd):
    mMp(mp),mRnd(rnd),N(mMp.size())
  {};

  virtual ~kCluster()
  {};

  //clustering into k clusters
  virtual double cluster(uint32_t k) = 0;
  virtual double cluster(uint32_t k, vector<MapPoint*>& csOpt)
  {
    double J=cluster(k);
    csOpt=mCs; // hand clustering outside
    return J;
  };

  // metric for measuring clustering quality
  virtual double silhouette(void) const;

  virtual const vector<MapPoint*>& getCentroids(void)
  {
    return mCs;
  };

  // assumes that the centroid is among the mCs stored with the class
  virtual vector<MapPoint*>& getCluster(uint32_t id)
  {
    return mClusters[id];
  };
  virtual vector<MapPoint*>& getCluster(MapPoint* centroid)
  {
    // find label of the provided centroid
    uint32_t clusterId=0;
    for(uint32_t i=0; i<mCs.size(); ++i)
      if(mCs[i]==centroid){
        clusterId=i;
        break;
      }
    //    cout<<" clusterId="<<clusterId<<endl;
    // retrieve cluster using label of centroid
    return mClusters[clusterId];
  };
//  virtual vector<MapPoint*>& getCluster(const vector<MapPoint*>&cs, MapPoint* centroid)
//  {
//    vector<uint32_t> l;
//    evalCost(cs); // get cluster lables
//    // find label of the provided centroid
//    uint32_t clusterId=0;
//    for(uint32_t i=0; i<cs.size(); ++i)
//      if(cs[i]==centroid){
//        clusterId=i;
//        break;
//      }
////    cout<<" clusterId="<<clusterId<<endl;
//    // retrieve cluster using label of centroid
//    return mClusters[clusterId];
////    vector<MapPoint*> cluster;
////    for(uint32_t i=0; i<mMp.size(); ++i)
////    {
////      if(l[i]==clusterId)
////      {
////        cluster.push_back(mMp[i]);
////      }
////    }
////    return cluster;
//  };

  void saveCentroidsToFile(char* pathToFile)
  {
    ofstream out(pathToFile);
    for (uint32_t i=0; i<k; ++i)
    {
      out<<mCs[i]->pBData->printForMatlab()<<endl;
    }
    out.close();
  };

  void loadCentroidsFromFile(char* pathToFile)
  {
    mCs.clear();
    ifstream in(pathToFile);
    bool readMore=true;
    while(readMore)
    {
      //cout<<"#Centroids="<<mCs.size()<<endl;
      MapPoint* mp=new MapPoint();
      BriefData* bd=new BriefData(mp);
      mp->pBData=bd;
      readMore=bd->readFromMatlab(in);
      if(readMore) mCs.push_back(mp);
    }
    in.close();
    cout<<"EvalCost with #Centroids="<<mCs.size()<<endl;
    k=mCs.size();

    evalCost(); // generate mClusters for fast access;
  };

protected:

  const vector<MapPoint*> mMp; // datapoints
  vector<MapPoint*> mCs; // current clustering centroids
  vector<vector<MapPoint*> > mClusters;
  Random& mRnd; // random number generator for clustering
  uint32_t k; // number of clusters
  uint32_t N; // number of datapoints

  // evaluate kClustering costfunction
  virtual double evalCost()
  {
    return 0;
  };
//  // evaluate kClustering costfunction and obtain labeling according to distance function
//  virtual double evalCost(const vector<MapPoint*>& cs, vector<uint32_t>& l) const = 0;

private:
  // declare private and do not implement -> no copy constructor
    kCluster& operator=(const kCluster&);
};

// TODO: Broken - do not use!
// kMedoids clustering using the PAM algorithm
class kMedoids : public kCluster
{
public:
  kMedoids(const vector<MapPoint*>& mp, Random& rnd): kCluster(mp,rnd)
  {};

  ~kMedoids()
  {};

  // kMedoids clustering into k clusters
  double cluster(uint32_t k);

protected:

  // evaluate kMedoids costfunction
  double evalCost();
};

class kBinaryMeans : public kCluster
{
public:
  kBinaryMeans(const vector<MapPoint*>& mp, Random& rnd): kCluster(mp,rnd)
  {};

  ~kBinaryMeans()
  {};

  virtual vector<MapPoint*>& getCluster(MapPoint* centroid)
  {
    return mClusters[mCentroidsLUT[centroid]];
  };

  // clustering into k clusters
  double cluster(uint32_t k_);

//  double clusterMultiThreaded(uint32_t k);

protected:
  map<MapPoint*,uint32_t> mCentroidsLUT; // Lookup table to find centroid ID quickly

private:
  // evaluate kBinaryMeans costfunction
  double evalCost();

//  double evalCostMultiThreaded(const vector<MapPoint*>& cs);
};


class LshTable
{
public:
  LshTable(Random& rnd, uint32_t hashLength) :
    mRnd(rnd),HASH_LENGTH(hashLength)
  {
    mG.reserve(HASH_LENGTH);
    generateHashFunction();
  };

  LshTable(Random& rnd, uint32_t hashLength, vector<double>& bitChangeProb) :
    mRnd(rnd),HASH_LENGTH(hashLength), mHashFctPrior(bitChangeProb), mHashFctSamplingPrior(bitChangeProb.size()+1,0.0)
  {
    mG.reserve(HASH_LENGTH);

    for(uint32_t i=0; i<mHashFctPrior.size(); ++i) mHashFctPrior[i]=1-mHashFctPrior[i]; // get bit stable probability
    double sum=0.0;
    for(uint32_t i=0; i<mHashFctPrior.size(); ++i) sum+=mHashFctPrior[i]; // normalize to 1
    for(uint32_t i=0; i<mHashFctPrior.size(); ++i) mHashFctPrior[i]/=sum; // normalize to 1
    cout<<"mHashFctSamplingPrior: "<<endl;
    for(uint32_t i=0; i<mHashFctPrior.size(); ++i)
    {
      mHashFctSamplingPrior[i+1]=mHashFctSamplingPrior[i]+mHashFctPrior[i]; // compute the integral.
      cout<<mHashFctSamplingPrior[i+1]<<" ";
    }; cout<<endl;

    generateHashFunction(mHashFctSamplingPrior);
  };

  void store(MapPoint* mp)
  {
    mT[mp->pBData->computeHash(mG)].push_back(mp);
  }

  const vector<MapPoint*>& get(MapPoint* mp)
  {
    return mT[mp->pBData->computeHash(mG)];
  }

  // number of buckets
  uint32_t numBuckets(){return mT.size();};

  // number of data elements in the buckets
  uint32_t numElems()
  {
    uint32_t s=0;
    for(map<uint32_t,vector<MapPoint*> >::iterator it=mT.begin(); it!=mT.end(); it++)
      s+=it->second.size();
    return s;
  }

  vector<uint32_t> getHashFunction()
  {
    vector<uint32_t> g(HASH_LENGTH,0);
    for(uint32_t i=0;i<HASH_LENGTH; ++i)
      g[i]=mG[i].first*8+mG[i].second;
    return g;
  }

private:
  Random& mRnd; // random number generator to generate hash function
  const uint32_t HASH_LENGTH;

  map<uint32_t,vector<MapPoint*> > mT; // hash table
  vector<pair<uint32_t,uint32_t> > mG; // hash function
  vector<double> mHashFctPrior;
  vector<double> mHashFctSamplingPrior;
  const static uint32_t SAMPLE_SIZE=10000;

  void generateHashFunction(vector<double>& hashFctSamplingPrior)
  {
    for(uint32_t i=0; i<HASH_LENGTH; ++i)
    {
      // draw without repetitions.
      bool alreadyDrawn=true;
      uint32_t g=0,g0=0,g1=0;

      while(alreadyDrawn)
      {
        alreadyDrawn=false;
        double s=double(mRnd(SAMPLE_SIZE))/double(SAMPLE_SIZE); // sample number between 0 and 1 with 1/SAMPLE_SIZE resolution
        for(uint32_t j=0; j<hashFctSamplingPrior.size()-1; ++j)
          if(hashFctSamplingPrior[j]<s && s<hashFctSamplingPrior[j+1]) g=j; // get bit number with the probabilities defined in hashFctSamplingPrior

        g0=g/8; g1=g%8;
        for(uint32_t j=0; j<mG.size(); ++j)
          alreadyDrawn |=  (g0==mG[j].first && g1==mG[j].second); // check that we are not drawing the same twice
      }
      mG.push_back(pair<uint32_t,uint32_t>(g0,g1));
//      cout<<g0<<" "<<g1<<endl;
    }
  }

  void generateHashFunction()
  {
    for(uint32_t i=0; i<HASH_LENGTH; ++i)
    {
      uint32_t g=mRnd.drawWithoutRepetition(BriefData::BRIEF_K*8, i==0);
      mG.push_back(pair<uint32_t,uint32_t>(g/8,g%8));
    }

//    const uint32_t BriefIndMax=BriefData::BRIEF_K/4;
//    for(uint32_t i=0; i<HASH_LENGTH; ++i)
//    {
//      // draw without repetitions.
//      bool alreadyDrawn=true;
//      pair<uint32_t,uint32_t> pairG;
//      while(alreadyDrawn)
//      {
//        alreadyDrawn=false;
//        pairG=pair<uint32_t,uint32_t>(mRnd(BriefIndMax),mRnd(32));
//        for(uint32_t j=0; j<mG.size(); ++j)
//          alreadyDrawn |= (pairG.first==mG[j].first && pairG.second==mG[j].second);// check that we are not drawing the same twice
//      }
//      mG.push_back(pairG);
////      cout<<" ("<<pairG.first<<";"<<pairG.second<<") ";
//    }
//    cout<<endl;
  }

//  uint32_t computeHash(BriefData* brief)
//  {
//    uint32_t hash=0;
//    for(uint32_t i=0; i<HASH_LENGTH; ++i)
//    {
//      hash |= (((brief->bd[mG[i].first] & (1<<mG[i].second))>>mG[i].second)<<i);
//    }
////    {// print the hashing process
////      cout<<"Brief: "<<(*brief)<<endl;
////      cout<<"Hash function: ";
////      for(uint32_t i=0; i<HASH_LENGTH; ++i)
////        cout<<mG[i].first<<"+"<<mG[i].second<<"  ";
////      char buf[40]; sprintf(buf,"%o",hash);
////      cout<<endl<<"Hash: "<<buf<<endl;
////    }
//    return hash;
//  }

  LshTable& operator=(const LshTable&);
};


class Classifier
{
public:
  Classifier(const vector<MapPoint*>& mapMPs) : mMPs(mapMPs)
  {};
  virtual ~Classifier()
  {};

  // set parameters (specific to the different classifiers
  virtual void setParams(uint32_t p1=0, uint32_t p2=0)
  {
    mP1=p1; mP2=p2;
  };
  // function that has to be called initially and after changing params
  // to ensure that the Classifier could prepare the dataset for classification
  virtual float prepare(void)
  {return 0.0;};
  virtual float prepare(uint32_t p1, uint32_t p2=0)
  {
    setParams(p1,p2);
    return prepare();
  };
  // find pairing (best match of curMPs in the mMPs)
  virtual float pair(vector<MapPoint*>& curMPs,
      vector<BriefAssociation>& pairing)
  {
//    pairing.resize(curMPs.size(),BriefAssociation(FLT_MAX,NULL,NULL));
    return 0.0;
  };

protected:
  const vector<MapPoint*>& mMPs; // MapPoints database which we try to match to
  uint32_t mP1,mP2;
private:
  // disable operator=
  Classifier& operator=(const Classifier&);
};


class NearestNeighbor : public Classifier
{
public:
  NearestNeighbor(const vector<MapPoint*>& mapMPs, uint32_t p1=0, uint32_t p2=0) : Classifier(mapMPs)
  {};
  ~NearestNeighbor()
  {};
  // no need to set params
  void setParams(uint32_t p1=0, uint32_t p2=0)
  {};
  //no need to prepare something
  float prepare(void)
  {return 0.0;};
  // do NN paring
  float pair(vector<MapPoint*>& curMPs, vector<BriefAssociation>& pairing);
  float pair(vector<MapPoint*>::const_iterator& queryMPBegin,
      vector<MapPoint*>::const_iterator& queryMPEnd,
      vector<BriefAssociation>::iterator& pairingBegin,
      vector<BriefAssociation>::iterator& pairingEnd);

  // do NN pairing of one querypoint
   void pair(MapPoint* queryMP, BriefAssociation& briefPair)
   {
     uint32_t distMin=INT_MAX;
     MapPoint *closestMP=NULL;
     BOOST_FOREACH(MapPoint* mp, mMPs)
     {
       if(mp->pBData!=NULL){
         uint32_t dist=queryMP->pBData->distBrief(*(mp->pBData));
         if(distMin > dist){
           distMin=dist;
           closestMP=mp;
         }
       }
   //    else{
         //TODO: why do I get NULLs?? -> initial KeyFrame?
   //      nullCount++;
   //    }
     }
     if(closestMP!=NULL)
     {
       if(briefPair.dist>distMin)
       {
         // replace stored pairing with current pairing
 //        queryMP->v3WorldPos=closestMP->v3WorldPos;
         briefPair=BriefAssociation(distMin,queryMP,closestMP);
       }
     }else{
       briefPair=BriefAssociation(INT_MAX,queryMP,NULL);
   //    nullCount++;
     }
     ASSERT(briefPair.dist<257, "dist="<<briefPair.dist<<" mp1="<<briefPair.mp1<<" mp2="<<briefPair.mp2);
   };
   void pair(MapPoint* queryMP, BriefAssociation& briefPair, uint32_t& retrieved)
   {
     retrieved=1;
     pair(queryMP, briefPair);
   };


   // do kNN pairing of own query point
   void pair(MapPoint* queryMP, vector<BriefAssociation>& briefPairing, uint32_t k)
   {
      briefPairing.clear();
      BOOST_FOREACH(MapPoint* mp, mMPs)
      {
        if(mp->pBData!=NULL){
          uint32_t dist=queryMP->pBData->distBrief(*(mp->pBData));
          briefPairing.push_back(BriefAssociation(dist,queryMP,mp));
        }
      }
      // sort only the first k elements -> the rest is unsorted!
      partial_sort(briefPairing.begin(), briefPairing.begin()+k, briefPairing.end());
   };

private:
};

// l=4 and k=5 is good for 175544 from looking at test of LSH
// l=3 and k=1 is almost NN for 175544 from looking at test of LSH
class LSH : public Classifier
{
public:
  LSH(const vector<MapPoint*>& mapMPs, uint32_t l=4, uint32_t k=5):
      Classifier(mapMPs), ml(l), mk(k), mBitChangeProb(0)
  {};
  LSH(vector<MapPoint*>& mapMPs, vector<double>& bitChangeProb, uint32_t l=4, uint32_t k=5):
      Classifier(mapMPs), ml(l), mk(k), mBitChangeProb(bitChangeProb)
  {};

  ~LSH()
  {
    for(size_t i=0; i<mLshTables.size(); ++i)
      delete mLshTables[i];
  };
  // set number of hash-tables and bit length of hash function
  void setParams(uint32_t p1, uint32_t p2)
  {
    ml=p1; mk=p2;
  }
  //build the hash-tables
  float prepare(void);
  // pair using prepared hash-tables
  float pair(vector<MapPoint*>& queryMPs,
        vector<BriefAssociation>& pairing)
  {
    vector<uint32_t> nRetrieved;
    return pair(queryMPs,pairing,nRetrieved);
  }

  float pair(vector<MapPoint*>& queryMPs,
          vector<BriefAssociation>& pairing, double& avgRetrieved)
    {
      vector<uint32_t> nRetrieved;
      float t=pair(queryMPs,pairing,nRetrieved);
      double N=0.0;
      for(uint32_t i=0; i<nRetrieved.size(); ++i)
        if (nRetrieved[i]!=0)
        {
          avgRetrieved+=nRetrieved[i];
          ++N;
        }
      avgRetrieved/=N; // compute average only over the queries that actually retrieved elements.
      return t;
    }

  float pair(vector<MapPoint*>& curMPs,
          vector<BriefAssociation>& pairing,vector<uint32_t>& nRetrieved);
  float pair(vector<MapPoint*>::const_iterator& queryMPBegin,
      vector<MapPoint*>::const_iterator& queryMPEnd,
      vector<BriefAssociation>::iterator& pairingBegin,
      vector<BriefAssociation>::iterator& pairingEnd);


  void pair(MapPoint* queryMP, BriefAssociation& pair, uint32_t& nRetrieved);
  void pair(MapPoint* queryMP, BriefAssociation& pair){
    uint32_t retrieved=0;
    this->pair(queryMP,pair,retrieved);
  };


private:
  uint32_t ml; // number of hashtables
  uint32_t mk; // bit length of hash function
  vector<double> mBitChangeProb;
  vector<LshTable*> mLshTables;
};

class kLSH : public Classifier
{
public:
  kLSH(const vector<MapPoint*>& mapMPs, uint32_t l=4, uint32_t m=5, uint32_t k=10):
      Classifier(mapMPs), ml(l), mm(m), mk(k), mBitChangeProb(0)
  {};
//  kLSH(vector<MapPoint*>& mapMPs, vector<double>& bitChangeProb, uint32_t l=4, uint32_t m=5, uint32_t k=10):
//      Classifier(mapMPs), ml(l), mm(m), mk(k), mBitChangeProb(bitChangeProb)
//  {};

  ~kLSH()
  {
    for(size_t i=0; i<mLshTables.size(); ++i)
      delete mLshTables[i];
  };
  // set number of hash-tables and bit length of hash function
  void setParams(uint32_t p1, uint32_t p2)
  {
    ml=p1; mm=p2;
  }
  //build the hash-tables
  float prepare(void);
  // pair using prepared hash-tables
  float pair(vector<MapPoint*>& curMPs,
        vector<BriefAssociation>& pairing)
  {
    vector<uint32_t> nRetrieved;
    return pair(curMPs,pairing,nRetrieved);
  }

  float pair(vector<MapPoint*>& curMPs,
          vector<BriefAssociation>& pairing, double& avgRetrieved)
    {
      vector<uint32_t> nRetrieved;
      float t=pair(curMPs,pairing,nRetrieved);
      double N=0.0;
      for(uint32_t i=0; i<nRetrieved.size(); ++i)
        if (nRetrieved[i]!=0)
        {
          avgRetrieved+=nRetrieved[i];
          ++N;
        }
      avgRetrieved/=N; // compute average only over the queries that actually retrieved elements.
      return t;
    }

  float pair(vector<MapPoint*>& curMPs,
          vector<BriefAssociation>& pairing,vector<uint32_t>& nRetrieved);

  void pair(MapPoint* queryMP, vector<BriefAssociation>& pair, uint32_t& nRetrieved);

private:
  uint32_t ml; // number of hashtables
  uint32_t mm; // bit length of hash function
  uint32_t mk; // how many matches to retrieve
  vector<double> mBitChangeProb;
  vector<LshTable*> mLshTables;
};

// kClusterClassifier is templated on the clustering algorithm
// it uses
template <class Cl>
class kClusterClassifier : public Classifier
{
public:
  kClusterClassifier(const vector<MapPoint*>& mapMPs):
      Classifier(mapMPs), mRnd(1), mKCluster(mMPs,mRnd)
    {
    };
  kClusterClassifier(const vector<MapPoint*>& mapMPs, uint32_t seed, uint32_t k=1):
      Classifier(mapMPs), mRnd(seed), mKCluster(mMPs,mRnd), mk(1)
    {
    };

  ~kClusterClassifier()
  {};
  // set number of centroids
  void setParams(uint32_t p1, uint32_t p2=0)
  {
    mk=p1;
  };
  // do the clustering
  float prepare(void)
  {
    // Clustering
    Timer t0;
    if(mKCluster.getCentroids().size()==0) mKCluster.cluster(mk);
    t0.toc();

//    cout<<"========== Clustering in "<<t0<<endl;

  //  vector<MapPoint*> centroids=mKCluster.getCentroids();
  //  for( uint32_t i=0; i<centroids.size(); ++i)
  //    cout<<*(centroids[i]->pBData)<<endl;
  //  cout<<"Silhouette: "<<mKCluster.silhouette()<<endl;
    return t0.lastDt();
  }
  // pair using the previously prepared centroids
  float pair(vector<MapPoint*>& queryMPs,
      vector<BriefAssociation>& pairing)
  {
    vector<uint32_t> nRetrieved;
    return pair(queryMPs,pairing,nRetrieved);
  };
  float pair(vector<MapPoint*>& queryMPs,
      vector<BriefAssociation>& pairing, double& avgRetrieved)
  {
    vector<uint32_t> nRetrieved;
    float t=pair(queryMPs,pairing,nRetrieved);
    double N=0.0;
    for(uint32_t i=0; i<nRetrieved.size(); ++i)
      if (nRetrieved[i]!=0)
      {
        avgRetrieved+=nRetrieved[i];
        ++N;
      }
    avgRetrieved/=N; // compute average only over the queries that actually retrieved elements.
    return t;
  };
  float pair(vector<MapPoint*>& queryMPs,
        vector<BriefAssociation>& pairing, vector<uint32_t>& nRetrieved)
  {
    pairing.clear();
    pairing.resize(queryMPs.size(),BriefAssociation(INT_MAX,NULL,NULL));
    nRetrieved.clear();
    nRetrieved.resize(queryMPs.size(),0);
    const vector<MapPoint*>& centroids=mKCluster.getCentroids();

    // find nearest neighbor
    uint32_t ind=0;
    Timer t0;
    BOOST_FOREACH(MapPoint* query, queryMPs)
    {
      // find nearest centroid
      uint32_t distMin=INT_MAX;
      MapPoint* closestCentroid=NULL;
      uint32_t id=0;
      BOOST_FOREACH(MapPoint* centroid, centroids)
      {
        uint32_t d=centroid->pBData->distBrief(*(query->pBData));
        if(distMin>d)
        {
          distMin=d;
          closestCentroid=centroid;
        }
        ++id;
      }
      vector<MapPoint*>& cluster=mKCluster.getCluster(closestCentroid);
      BriefAssociation knn; // k nearest neighbors of cur/mp!
      NearestNeighbor NN(cluster);
      NN.pair(query, knn);
      pairing[ind]=knn;
      nRetrieved[ind]=cluster.size();
      ++ind;
    }
    return t0.toc();
  };

  Cl& getClusterer(void)
  {
    return mKCluster;
  }

private:
  Random mRnd; // random number generator
  Cl mKCluster; // Clustering class - has the code for clustering
  uint32_t mk; // number of centroids
};

template <class Cl>
class LSHkClusterClassifier : public Classifier
{
public:

  LSHkClusterClassifier(const vector<MapPoint*>& mapMPs, uint32_t seed=1,
      uint32_t k=1, uint32_t m=5, uint32_t l=4) :
      Classifier(mapMPs), mRnd(seed), mKCluster(mMPs,mRnd),
      mpLSH(NULL), mk(k), mm(m), ml(l)
    {
    };

  ~LSHkClusterClassifier()
  {
    delete mpLSH;
  };
  // set number of centroids
  void setParams(uint32_t p1, uint32_t p2=0)
  {
    assert(false); // do not use!
//    mk=p1;
  };
  // do the clustering
  float prepare(void)
  { // Clustering
    Timer t0;
    if(mKCluster.getCentroids().size()==0) mKCluster.cluster(mk);
    mpLSH=new LSH(mKCluster.getCentroids(), ml, mm);
    mpLSH->prepare();
    t0.toc();
    return t0.lastDt();
  }
  // pair using the previously prepared centroids
  float pair(vector<MapPoint*>& curMPs,
      vector<BriefAssociation>& pairing)
  {
    vector<uint32_t> nRetrieved;
    return pair(curMPs,pairing,nRetrieved);
  }
  float pair(vector<MapPoint*>& curMPs,
      vector<BriefAssociation>& pairing, double& avgRetrieved)
  {
    vector<uint32_t> nRetrieved;
    float t=pair(curMPs,pairing,nRetrieved);
    double N=0.0;
    for(uint32_t i=0; i<nRetrieved.size(); ++i)
      if (nRetrieved[i]!=0)
      {
        avgRetrieved+=nRetrieved[i];
        ++N;
      }
    avgRetrieved/=N; // compute average only over the queries that actually retrieved elements.
    return t;
  }
  float pair(vector<MapPoint*>& queryMPs,
        vector<BriefAssociation>& pairing, vector<uint32_t>& nRetrieved)
  {
    pairing.clear();
    pairing.resize(queryMPs.size(),BriefAssociation(FLT_MAX,NULL,NULL));
    nRetrieved.clear();
    nRetrieved.resize(queryMPs.size(),0);

    cout<<"LSH to find closest Centroids"<<endl;
    Timer t0;
    vector<BriefAssociation> pairingLSH(queryMPs.size(),BriefAssociation(0.0,NULL,NULL));
    mpLSH->pair(queryMPs,pairingLSH,nRetrieved);
    for(uint32_t i=0; i<pairingLSH.size(); ++i)
    {
      MapPoint* closestCentroid=NULL;
//      cout<<pairingLSH[i].mp1<<" "<<pairingLSH[i].mp2<<endl;
      if(pairingLSH[i].mp2==NULL)
      { // fallback to standard NN
        vector<MapPoint*> centroids=mKCluster.getCentroids();
        vector<MapPoint*> cur; cur.push_back(pairingLSH[i].mp1);
        vector<BriefAssociation> knn; // k nearest neighbors of cur/mp!
        NearestNeighbor NN(centroids);
        NN.pair(cur, knn);
        closestCentroid=knn[0].mp2;
      }else{
        closestCentroid=pairingLSH[i].mp2;
      }
      vector<MapPoint*> cluster=mKCluster.getCluster(closestCentroid);
      vector<MapPoint*> cur; cur.push_back(pairingLSH[i].mp1);
      vector<BriefAssociation> knn; // k nearest neighbors of cur/mp!
      NearestNeighbor NN(cluster);
      NN.pair(cur, knn);
      pairing[i]=knn[0];
      nRetrieved[i]=cluster.size();
    }
    t0.toc();
//    cout<<"========== Finding Neighbors using Centroids "<<t0<<endl;
    return t0.lastDt();
  }

  Cl& getClusterer(void)
  {
    return mKCluster;
  }

private:
  Random mRnd; // random number generator
  Cl mKCluster; // Clustering class - has the code for clustering
  LSH* mpLSH; // for faster access to the centroids
  uint32_t mk; // number of centroids
  uint32_t mm; // hash code length
  uint32_t ml; // number of hash tables

};

//class Classifier_impl
//{
//public:
//  static void NearestNeighbor(vector<MapPoint*>& curMPs, vector<MapPoint*>& mapMPs,
//      vector<BriefAssociation>& pairing);
//
//  static void LSH(vector<MapPoint*>& curMPs, vector<MapPoint*>& mapMPs,
//        vector<BriefAssociation>& pairing, uint32_t l=0, uint32_t k=0);
//
//  static void kMedoid(vector<MapPoint*>& curMPs, vector<MapPoint*>& mapMPs,
//        vector<BriefAssociation>& pairing, uint32_t k=10);
//
//  static void kBinaryMean(vector<MapPoint*>& curMPs, vector<MapPoint*>& mapMPs,
//        vector<BriefAssociation>& pairing, uint32_t k=100);
//};


#endif /* CLASSIFIER_HPP_ */
