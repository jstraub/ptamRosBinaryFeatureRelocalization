/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE testBRIEF
#include <boost/test/unit_test.hpp>

#include <MapPoint.h>
#include <BriefData.hpp>
#include <Classifier.hpp>
#include <Random.hpp>
#include <timer.hpp>

#define USE_LSH // use kBinMeans with LSH for Means retrieval
#ifdef USE_LSH
const string outputPath("../lshKBinMeansResults/");
#else
const string outputPath("../kBinMeansResults/");
#endif

#include "ThreadedClassifier.hpp"
#include "helpers.hpp"

#include <boost/smart_ptr/shared_ptr.hpp>

#include <fstream>
#include <stdio.h>
#include <vector>
#include <map>
#include <time.h>

using namespace std;

BOOST_AUTO_TEST_CASE(BRIEFClusteringKBinaryMeans)
{
  //const static std::string pathToData("/home/straub/workspace_navvis/data/20120303_093952_Start0_Skip0_Reloc0_NFrames9999_Processed_20120407_094708");
  //  const static std::string pathToData("/home/straub/workspace_navvis/data/20120420_175544_Start2_Skip0_Reloc0_NFrames9999_Processed_20120430_174916");
  //  const static std::string pathToData("/home/straub/workspace_navvis/data/20120420_175544_Start2_Skip0_Reloc0_NFrames9999_Processed_20120503_110414");
//  const static std::string pathToData("/home/straub/workspace_navvis/data/20120420_175544_Start2_Skip0_Reloc0_Type1_NFrames9999_Processed_20120510_090602");
  const static std::string pathToData("/home/straub/workspace_navvis/data/20120420_175544_Start2_Skip0_Reloc0_Type1_NFrames9999_Processed_20120514_083455");

  std::vector<size_t> keyFrameNrs;
  std::vector<BriefData*> keyBriefs;
  std::map<BriefData*,size_t> keyBriefFrames;
  std::vector<BriefData*> briefs;
  std::map<BriefData*,size_t> briefFrames;

  if(!loadKeyFrameNrs(pathToData,keyFrameNrs)) return;
  if(!loadBriefs(pathToData,"allKeyBriefDescriptors.txt",keyBriefs, keyBriefFrames)) return;
  if(!loadBriefs(pathToData,"briefDescriptors.txt",briefs, briefFrames)) return;

  Random rnd(time(NULL));
  uint32_t M=550, N=keyBriefs.size(), Nall=briefs.size();
  uint32_t Ntests=100;
  uint32_t KMax=5200, k0=10, kStep=10;
  uint32_t nThreadsMax=8;
  uint32_t K=floor((KMax-k0)/kStep)+1;

  vector<MapPoint*> queryMPs(M,NULL); // query MPs
  vector<MapPoint*> keyMPs(N,NULL); // MPs from  KeyFrames
  vector<MapPoint*> allMPs(Nall,NULL); // MPs from frames after KeyFrames

  {// connect BriefData to MapPoints and select random subset from allMps for query
    for(size_t i=0; i<N; ++i)
      keyMPs[i]=(&(keyBriefs[i]->pt));
    for(size_t i=0; i<Nall; ++i)
      allMPs[i]=(&(briefs[i]->pt));
  }

  vector<kBinMeansMulti<kBinaryMeansClassifier > *> threads;
  uint32_t nThreadsRun=0;
  vector<double> tPrep(K,0.0);
  vector<bool> threadRunning(K,false);
  uint32_t k=k0;
  bool threadsRunning=true;
  while(k<=KMax || threadsRunning)
  {// start threads
    if(k<=KMax && nThreadsRun<nThreadsMax)
    { // spawn next thread if less than nThreadMax are running
      threads.push_back(new kBinMeansMulti<kBinaryMeansClassifier >(keyMPs,k));
      threadRunning[threads.size()-1]=true;
      cout<<"spawned thread for k="<<k<<endl;
      ++nThreadsRun;
      k+=kStep;
    }
    threadsRunning=false;
    for (uint32_t i=0; i<threads.size(); ++i)
    { // check for finished threads
      bool running=threads[i]->isRunning();
      if(threadRunning[i] && !running)
      {
        cout<<"Thread "<<i<<" finished."<<endl;
        tPrep[i]=threads[i]->getTPrep();
        threadRunning[i]=false;
        --nThreadsRun;
      }
      threadsRunning |= running;
    }
  }

  double tNNPair=0.0;
  vector<double> tPair(K,0.0);
  vector<double> avgDist(K,0.0);
  vector<vector<double> > notPaired(K,vector<double>(Ntests,0.0));
  vector<vector<double> > tp(K,vector<double>(Ntests,0.0)); // # LSH and NN have same pairing
  vector<vector<double> > nRetrieved(K,vector<double>(Ntests,0.0));

  for(size_t n=0; n<Ntests; ++n)
  {
    cout<<"Generating query"<<endl;
    for(size_t i=0; i<M; ++i)
    {
      uint32_t j=rnd(Nall);
      if(queryMPs[i]!=NULL)
      {
        if(queryMPs[i]->pBData!=NULL) delete queryMPs[i]->pBData;
        delete queryMPs[i];
      }
      queryMPs[i]=new MapPoint();
      queryMPs[i]->pBData= new BriefData(queryMPs[i]);
      queryMPs[i]->pBData->copyDataFrom(allMPs[j]->pBData);
    }
    cout<<"Computing NN"<<endl;
    vector<BriefAssociation> pairingNN;
    {// generate NN groundtruth
      NearestNeighbor NN(keyMPs);
      tNNPair+=NN.pair(queryMPs,pairingNN);
    }

    for(uint32_t k=0; k<threads.size(); ++k)
    {
      vector<BriefAssociation> pairingKBinMeans;
      vector<uint32_t> retrieved(M,0);
      cout<<"Pairing kBinMeans k="<<k<<" von "<<threads.size()<<" or "<<K<<endl;
      tPair[k]+=threads[k]->getKBinMeans().pair(queryMPs,pairingKBinMeans,retrieved); // query
      //      printPairing(briefFrames,pairingKBinMeans);
      cout<<"compute statistics"<<endl;
      double avgDist_=0.0; uint32_t notPaired_=0, tp_=0;
      evalPairing(pairingKBinMeans, avgDist_, notPaired_);
      compairAgainstNN(pairingKBinMeans,pairingNN, tp_);

      avgDist[k]+=avgDist_; cout<<"avgDist_="<<avgDist_<<endl;
      notPaired[k][n]=notPaired_; cout<<"notPaired_="<<notPaired_<<endl;
      nRetrieved[k][n]=0;
      for (uint32_t j=0; j<M; ++j)
        nRetrieved[k][n]+=retrieved[j];
      tp[k][n]=tp_; cout<<"tp_="<<tp_<<endl;
    }
  }

  cout<<"Writing to files"<<endl;

  string outPath=outputPath;
  ofstream outAvgDist(outPath.append("kBMAvgDist.txt").c_str()); outPath=outputPath;
  ofstream outNotPaired(outPath.append("kBMNotPaired.txt").c_str()); outPath=outputPath;
  ofstream outTimePair(outPath.append("kBMTimePair.txt").c_str()); outPath=outputPath;
  ofstream outTruePos(outPath.append("kBMTruePos.txt").c_str()); outPath=outputPath;
  ofstream outAvgRetr(outPath.append("kBMAvgRetrieved.txt").c_str());

  for (uint32_t k=0; k<threads.size(); ++k)
  {
    outTimePair<<tPair[k]/Ntests<<"\t";
    outAvgDist<<avgDist[k]/Ntests<<"\t";
    for (uint32_t n=0; n<Ntests; ++n)
    {
      outNotPaired<<notPaired[k][n]<<"\t";
      outTruePos<<tp[k][n]<<"\t";
      outAvgRetr<<nRetrieved[k][n]<<"\t";
    }
    outNotPaired<<endl;
    outTruePos<<endl;
    outAvgRetr<<endl;
  }
  outTimePair.close();
  outAvgDist.close();
  outNotPaired.close();
  outTruePos.close();
  outAvgRetr.close();

  outPath=outputPath;
  ofstream outTimeNNPair(outPath.append("kBMTimeNNPair.txt").c_str());
  outTimeNNPair<<tNNPair/Ntests<<endl;
  outTimeNNPair.close();

  //start matlab for plotting the results
  //system("matlab -nosplash -nodesktop -r \"addpath /home/straub/workspace_navvis/matlab/; plotKBinMeansResults\"; stty echo;");

  return;

  // ----------------------------------------- kBinaryMeans
//  ofstream outAvgDist("../kBinMeansResults/kBMAvgDist.txt");
//  ofstream outNotPaired("../kBinMeansResults/kBMNotPaired.txt");
//  ofstream outTimePair("../kBinMeansResults/kBMTimePair.txt");
//  ofstream outTruePos("../kBinMeansResults/kBMTruePos.txt");
//  for(uint32_t k=10; k<=1000; k+=10)
//  {
//    vector<BriefAssociation> pairingKBinMeans;
//    kClusterClassifier<kBinaryMeans> kBinMeans(keyMPs);
//
//    char buf[100]; sprintf(buf,"../kBinMeansResults/%04dMeans.txt",k);
//    ifstream in(buf);
//    if(in)
//    {// if the file already exists (ie we already computed centroids) avoid recomputing
//      kBinMeans.getClusterer().loadCentroidsFromFile(buf);
//    }else{ // compute centroids
//      float tPrep=dynamic_cast<Classifier*>(&kBinMeans)->prepare(k); // clustering
//      cout<<"#Centroids="<<kBinMeans.getClusterer().getCentroids().size()<<endl;
//      kBinMeans.getClusterer().saveCentroidsToFile(buf); //save for next time
//      sprintf(buf,"../kBinMeansResults/kBM%05dMeansTimePrep.txt",k);
//      ofstream outTimePrep(buf);
//      outTimePrep<<tPrep<<endl;
//      outTimePrep.close();
//    }
//    float tPair=kBinMeans.pair(queryMPs,pairingKBinMeans); // query
//    // print some stuff
//    cout<<"--------- Time taken by NN: "<<tPair<<endl;
//    printPairing(briefFrames,pairingKBinMeans);
//    // compute statistics
//    double avgDist; uint32_t notPaired;
//    evalPairing(pairingKBinMeans, avgDist, notPaired);
//    uint32_t tp=0; // # LSH and NN have same pairing
//    compairAgainstNN(pairingKBinMeans,pairingNN, tp);
//    // save to file
//    outTimePair<<tPair<<"\t";
//    outAvgDist<<avgDist<<"\t";
//    outNotPaired<<notPaired<<"\t";
//    outTruePos<<tp<<"\t";
//  }
//
//
//  {// test saved centroids
//    kClusterClassifier<kBinaryMeans> kBMloaded(keyMPs);
//    kClusterClassifier<kBinaryMeans> kBMcomp(keyMPs);
//    uint32_t k=50;
//    char buf[100]; sprintf(buf,"../kBinMeansResults/%04dMeans.txt",k);
//    kBMloaded.getClusterer().loadCentroidsFromFile(buf);
//
//    float tPrep=dynamic_cast<Classifier*>(&kBMcomp)->prepare(k); // clustering
//    cout<<"#Centroids="<<kBMcomp.getClusterer().getCentroids().size()<<endl;
//
//    BOOST_FOREACH(MapPoint* mpLoad, kBMloaded.getClusterer().getCentroids())
//    {
//      BOOST_FOREACH(MapPoint* mpComp, kBMcomp.getClusterer().getCentroids())
//      {
//        cout<<mpLoad->pBData->distBrief(*(mpComp->pBData))<< " ";
//      }
//      cout<<endl;
//    }
//  }
//
//  {// empty memory
//    for(size_t i=0; i<keyBriefs.size(); ++i)
//    {
//      delete &(keyBriefs[i]->pt);
//      delete keyBriefs[i];
//    }
//  }
}

