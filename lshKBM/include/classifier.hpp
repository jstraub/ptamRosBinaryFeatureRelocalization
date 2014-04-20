/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 
#pragma once

#include <timer.hpp>
#include <MultiThreading.hpp>
#include <association.hpp>

#include <stdint.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

/* Parameters for a Classifier
 *
 */
struct Params
{
  Params() : p(0)
  {};
  Params(const vector<uint32_t>& p_) : p(p_)
  {};
  Params(const Params& clP) : p(clP.p)
  {};
  vector<uint32_t> p;

  string dump() const
  {
    stringstream ss;
    for(uint32_t i=0; i<p.size(); ++i)
      ss<<p[i]<<" ";
    return ss.str();
  }
};

/* Common classifier results that are needed for evaluation purposes
 *
 */
struct ClassifierResult
{
  ClassifierResult() :
    tPrep(0.0), tPair(0.0),
    avgDist(0.0), paired(0), tp(0)
  {};

  ClassifierResult(uint32_t nQueries) :
    retrieved(nQueries,0), supports(nQueries,0), tPrep(0.0), tPair(0.0),
    avgDist(0.0), paired(0), tp(0)
  {};

  vector<uint32_t> retrieved; // number of features retrieved per query
  vector<uint32_t> supports; // number of hashtables that support the aNN
  float tPrep; // time for preparation
  float tPair; // time for pairing
  double avgDist; //average distance between query and match
  uint32_t paired; // actually number of paired features -> might be bigger than the number of queried features
  uint32_t nFoundPairings; // number of queries for which one or more pairings was found
  uint32_t tp; // number of true positives

  uint32_t querySize; // how many did we query
  uint32_t mapSize; // how many were in the map

  uint32_t sumRetrieved() const
  {
    uint32_t sum=0;
    for (uint32_t j=0; j<retrieved.size(); ++j)
      sum+=retrieved[j];
    return sum;
  };

  double stdSupport() const
  {
    double mean=double(sumSupport())/double(supports.size());
    uint32_t sum=0;
    for (uint32_t j=0; j<supports.size(); ++j)
      sum+=(supports[j]-mean)*(supports[j]-mean);
    return sqrt(double(sum)/double(supports.size()-1));
  }

  double avgSupport() const
  {
    return double(sumSupport())/double(supports.size());
  };
  uint32_t sumSupport() const
  {
    uint32_t sum=0;
    for (uint32_t j=0; j<supports.size(); ++j)
      sum+=supports[j];
    return sum;
  };
};


template<class Desc, class Dist>
class Classifier
{
public:
  Classifier(const Params& clP)
    : mMapDs(0), mClP(clP)
  {};
  Classifier(const vector<Desc*>& mapDs, const Params& clP)
    : mMapDs(mapDs), mClP(clP)
  {};
  Classifier(const Classifier<Desc,Dist>& cl)
    : mMapDs(cl.getMapDs()), mClP(cl.getParams())
  {};
  virtual ~Classifier()
  {};

  // set parameters (specific to the different classifiers
  virtual void setParams(const Params& clP)
  {
    mClP=clP;
  };
  // function that has to be called initially and after changing params
  // to ensure that the Classifier could prepare the dataset for classification
  virtual float prepare(void) =0;

  virtual float prepare(const Params& clP)
  {
    setParams(clP);
    return prepare();
  };

  // returns a vector of associations since, there might be several
  // pairings with the same lowest distance
  virtual bool pair(Desc* queryMP, vector<Assoc<Desc,Dist> >& pair)=0;
  virtual bool pair(Desc* queryMP, vector<Assoc<Desc,Dist> >& pair, uint32_t& nRetrieved)=0;
  // for k-nn
  virtual void kPair(Desc* queryMP, vector<Assoc<Desc,Dist> >& annPair, uint32_t k)=0;
  virtual void kPair(Desc* queryMP, vector<Assoc<Desc,Dist> >& annPair, uint32_t& nRetrieved, uint32_t k)=0;
  virtual float pair(const vector<Desc*>& queryDs, vector<Assoc<Desc,Dist> >& pairing)
  {
    pairing.reserve(queryDs.size());
    Timer t0;
    for(uint32_t i=0; i<queryDs.size(); ++i)
    { // the results of this query are simply appended to briefPair
      pair(queryDs[i],pairing);
    }
    return t0.toc();
  };
  virtual float pair(const vector<Desc*>& queryDs, vector<Assoc<Desc,Dist> >& pairing, vector<uint32_t>& nRetrieved)
  {
    pairing.reserve(queryDs.size());
    nRetrieved.resize(queryDs.size(),0);
    Timer t0;
    for(uint32_t i=0; i<queryDs.size(); ++i)
    { // the results of this query are simply appended to briefPair
      pair(queryDs[i],pairing,nRetrieved[i]);
    }
    return t0.toc();
  };

  virtual float pair(const vector<Desc*>& queryDs, vector<Assoc<Desc,Dist> >& pairing, vector<uint32_t>& nRetrieved, uint32_t& nFoundPairings)
  {
    nFoundPairings=0;
    pairing.reserve(queryDs.size());
    nRetrieved.resize(queryDs.size(),0);
    Timer t0;
    for(uint32_t i=0; i<queryDs.size(); ++i)
    { // the results of this query are simply appended to briefPair
      uint32_t nBefore=pairing.size();
      pair(queryDs[i],pairing,nRetrieved[i]);
      if(nBefore<pairing.size())
      { // found one or more matches for this query feature
        ++nFoundPairings;
//        if(nBefore+1<pairing.size())
//        { // check that the ones with equal distance are not the same!
//          for(uint32_t i=0; i<pairing.size(); ++i)
//            for(uint32_t j=0; j<pairing.size(); ++j)
//              assert(pairing[i].m!=pairing[j].m);
//        }

      }
    }
    return t0.toc();
  };

//  virtual float pair(const vector<Desc*>& queryDs, vector<Assoc<Desc,Dist> >& pairing, double& avgRetrieved)
//  {
//    vector<uint32_t> nRetrieved;
//    float t=pair(queryDs, pairing, nRetrieved);
//    double N=0.0;
//    for(uint32_t i=0; i<nRetrieved.size(); ++i)
//      if (nRetrieved[i]!=0){
//        avgRetrieved+=nRetrieved[i];
//        ++N;
//      }
//    avgRetrieved/=N; // compute average only over the queries that actually retrieved elements.
//    return t;
//  };
//  bool pair(Desc* queryMP, Assoc<Desc,Dist>& pair)
//  {
//    vector<Assoc<Desc,Dist> > pairings;
//    bool ret=this->pair(queryMP, Assoc<Desc,Dist> pairings);
//    pair=pairings[0]; // simply choose first one even if there are multiple
//    return ret;
//  }
  virtual float pair(const typename vector<Desc*>::const_iterator& queryDBegin,
      const typename vector<Desc*>::const_iterator& queryDEnd,
      const typename vector<Assoc<Desc,Dist> >::iterator& pairingBegin,
      const typename vector<Assoc<Desc,Dist> >::iterator& pairingEnd)
  {
    typename vector<Assoc<Desc,Dist> >::iterator itPair=pairingBegin;
    Timer t0;
    for(typename vector<Desc*>::const_iterator itQuery=queryDBegin;
        itQuery!=queryDEnd;
        itQuery++, itPair++)
    {
      assert(*itQuery!=NULL);

      vector<Assoc<Desc,Dist> > pairings;
      this->pair(*itQuery, pairings);
      if(pairings.empty())
      {
        *itPair=Assoc<Desc,Dist>(*itQuery,NULL);
      }else{
        *itPair=pairings[0]; // simply choose first one even if there are multiple
      }
      //      this->pair(*itQuery,*itPair);
      assert((*itPair).q!=NULL);
    }
    return t0.toc();
  }
  virtual float kPair(const vector<Desc*>& queryDs, vector<Assoc<Desc,Dist> >& pairing, vector<uint32_t>& nRetrieved, uint32_t k)
  {
    pairing.reserve(queryDs.size()*k); // reserve max possible size
    nRetrieved.resize(queryDs.size(),0);
    Timer t0;
    // find nearest neighbor
    vector<Assoc<Desc,Dist> > knn;
    for(uint32_t i=0; i<queryDs.size(); ++i)
    {
      kPair(queryDs[i],knn,nRetrieved[i],k);
      for(uint32_t i=0; i < min(uint32_t(knn.size()),k); ++i)
        pairing.push_back(knn[i]); //[ind*mLshParams.getK()+i]=knn[i]; // k Nearest neighbors
    }
    return t0.toc();
  };
  virtual float kPair(const vector<Desc*>& queryDs, vector<Assoc<Desc,Dist> >& pairing, uint32_t k)
  {
    vector<uint32_t> nRetrieved;
    return kPair(queryDs, pairing, nRetrieved, k);
  };
  virtual float kPair(const vector<Desc*>& queryDs, vector<vector<Assoc<Desc,Dist> > >& pairing, vector<uint32_t>& nRetrieved, uint32_t k)
  {
    pairing.resize(queryDs.size(),vector<Assoc<Desc,Dist> >()); // reserve max possible size
    nRetrieved.resize(queryDs.size(),0);
    Timer t0;
    // find nearest neighbor
    vector<Assoc<Desc,Dist> > knn;
    for(uint32_t i=0; i<queryDs.size(); ++i)
    {
      kPair(queryDs[i],pairing[i],nRetrieved[i],k);
      pairing[i].resize(min(uint32_t(knn.size()),k),Assoc<Desc,Dist>());
    }
    return t0.toc();
  };
  virtual float kPair(const vector<Desc*>& queryDs, vector<vector<Assoc<Desc,Dist> > >& pairing, uint32_t k)
  {
    vector<uint32_t> nRetrieved;
    return kPair(queryDs, pairing, nRetrieved, k);
  };

  // support evaluation
  virtual bool pair(Desc* queryD, Assoc<Desc,Dist>& annPair, uint32_t& nRetrieved, uint32_t& support)=0;
  virtual float pair(const vector<Desc*>& queryDs, vector<Assoc<Desc,Dist> >& pairing, vector<uint32_t>& nRetrieved, vector<uint32_t>& supports)
  {
    supports.resize(queryDs.size(),0);
    pairing.reserve(queryDs.size());
    nRetrieved.resize(queryDs.size(),0);
    Timer t0;
    Assoc<Desc,Dist> match;
    for(uint32_t i=0; i<queryDs.size(); ++i)
    {
      if(pair(queryDs[i], match, nRetrieved[i],supports[i])) pairing.push_back(match);
    }
    return t0.toc();
  };

  const vector<Desc*>& getMapDs() const {return mMapDs;};
  const Params& getParams() const {return mClP;};

protected:
  const vector<Desc*>& mMapDs; // MapPoints database which we try to match to
  Params mClP;

private:
  // disable operator=
  Classifier& operator=(const Classifier&);
};


template<typename Desc, typename Dist>
class ClassifierThreaded : public SingleThread<void>
{
public:
//  ClassifierThreaded(const vector<Desc*>& mapMPs,
//      typename vector<Desc*>::const_iterator& queryMPBegin,
//      typename vector<Desc*>::const_iterator& queryMPEnd,
//      typename vector<Assoc<Desc,Dist> >::iterator& pairingBegin,
//      typename vector<Assoc<Desc,Dist> >::iterator& pairingEnd,
//      const ClassifierParams& clP)
//  : SingleThread<void>(),
//    mpCl(new Cl(mapMPs,clP)),
//    mQueryMPBegin(queryMPBegin), mQueryMPEnd(queryMPEnd),
//    mPairingBegin(pairingBegin), mPairingEnd(pairingEnd),
//    mClIsManagedByThis(true)
//    {};

  ClassifierThreaded(Classifier<Desc,Dist>* pCl,
      const typename vector<Desc*>::const_iterator& queryMPBegin,
      const typename vector<Desc*>::const_iterator& queryMPEnd,
      const typename vector<Assoc<Desc,Dist> >::iterator& pairingBegin,
      const typename vector<Assoc<Desc,Dist> >::iterator& pairingEnd)
  : SingleThread<void>(),
    mpCl(pCl),
    mQueryMPBegin(queryMPBegin), mQueryMPEnd(queryMPEnd),
    mPairingBegin(pairingBegin), mPairingEnd(pairingEnd),
    mClIsManagedByThis(false)
    {};

  virtual ~ClassifierThreaded()
  {
    if(mClIsManagedByThis){
      assert(false);
      delete mpCl;
    }
  };

protected:
  void doWork_impl()
  {
    assert(mpCl!=NULL);
    if(mClIsManagedByThis) mpCl->prepare();
    mpCl->pair(mQueryMPBegin,mQueryMPEnd,mPairingBegin,mPairingEnd);
  };

private:

  Classifier<Desc,Dist>* mpCl;

  typename vector<Desc*>::const_iterator mQueryMPBegin;
  typename vector<Desc*>::const_iterator mQueryMPEnd;
  typename vector<Assoc<Desc,Dist> >::iterator mPairingBegin;
  typename vector<Assoc<Desc,Dist> >::iterator mPairingEnd;

  bool mClIsManagedByThis;
};

template<class Desc, class Dist>
void compairAgainstNN(const vector<Assoc<Desc,Dist> >& pairing,
    const vector<Assoc<Desc,Dist> >& pairingNN,
    uint32_t& tp)
{// compare LSH pairing against NN (ground truth?)
//  cout<<"compare LSH pairing against NN (ground truth?)"<<endl;
  tp=0;
  for(size_t i=0; i<pairingNN.size();++i)
  {
    assert(pairingNN[i].m!=NULL);
    for(size_t j=0; j<pairing.size();++j)
      if((pairingNN[i].q==pairing[j].q) && (pairingNN[i].m==pairing[j].m))
      {
        ++tp;
        break;
      }
  }
//  cout<<"#LSH matched equal to NN="<<nEqual<<" #LSH matched differently from NN="<<nDiff<< " (total#="<<pairingNN.size()<<")"<<endl;
}

template<class Desc, class Dist>
void evalPairing(const vector<Assoc<Desc,Dist> >& pairing, double& avgDist)
{
  double sumDist=0, N=0;
  for(size_t i=0; i<pairing.size();++i)
    if(pairing[i].m!=NULL){
      sumDist+=pairing[i].d;
      ++N;
    }
  avgDist=sumDist/N;
}

// ClassifierThreadedEval for evaluation purposes
template<class Desc, class Dist>
class ClassifierThreadedEval : public SingleThread<ClassifierResult>
{
public:
  ClassifierThreadedEval(Classifier<Desc,Dist>* pCl, const vector<Desc*>& queryMPs,
      const vector<Assoc<Desc,Dist> >& pairingNN, uint32_t kNN=1)
  : SingleThread<ClassifierResult>(),
    mpCl(pCl), mQueryMPs(queryMPs),
      mPairingNN(pairingNN), mkNN(kNN)
  {};
  ~ClassifierThreadedEval()
  {
    delete mpCl;
  };

  const Classifier<Desc,Dist>& getClassifier() {return mpCl;};

private:

  Classifier<Desc,Dist>* mpCl; // classifier
  const vector<Desc*>& mQueryMPs;
  const vector<Assoc<Desc,Dist> >& mPairingNN; // groundtruth pairing
  const uint32_t mkNN; // number of NN will be evaluated

  void doWork_impl()
  {// work in here
    vector<Assoc<Desc,Dist> > pairingLSH;
    float tPrep=mpCl->prepare();
    vector<uint32_t> retrieved;
    vector<uint32_t> supports;
    uint32_t nFoundPairings=0;
    float tPair=0.0;
    if(mkNN>1)
      tPair=mpCl->kPair(mQueryMPs,pairingLSH,retrieved,mkNN);
    else
      tPair=mpCl->pair(mQueryMPs,pairingLSH,retrieved, nFoundPairings);//,supports); // only works for LSH!!

    // print some stuff
    //        cout<<"--------- Time taken by LSH: "<<tPair<<"ms"<<endl;
    //        printPairing(briefFrames,pairingLSH);
    // compute statistics
    boost::mutex::scoped_lock l(mResMutex); // lock before assigning values
    mRes.tPrep=tPrep;
    mRes.tPair=tPair;
    mRes.retrieved=retrieved;
    mRes.supports=supports;
    mRes.paired=pairingLSH.size();
    mRes.nFoundPairings=nFoundPairings;
    evalPairing<Desc,Dist>(pairingLSH, mRes.avgDist);
    compairAgainstNN<Desc,Dist>(pairingLSH, mPairingNN, mRes.tp);
    cout<<endl
        <<"Classifier thread ("<<mpCl->getParams().dump()<<") stopped:"
        <<"\t#tp="<<mRes.tp <<"\tavgDist="<<mRes.avgDist<< "\tsumRetr="<<mRes.sumRetrieved()
        <<"\t#paired="<<mRes.paired
        <<"\tsupport="<<mRes.avgSupport()<<" +-"<<mRes.stdSupport()<<endl;
//    usleep(500000);
  };
};
