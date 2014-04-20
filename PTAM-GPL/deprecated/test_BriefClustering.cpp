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

//#include <TrackerData.h>
//#include <MapMaker.h>
//#include <KeyFrame.h>
#include <MapPoint.h>
#include <BriefData.hpp>
#include <Classifier.hpp>

#include <timer.hpp>

#include <fstream>
#include <stdio.h>
#include <vector>
#include <map>

#include "helpers.hpp"

using namespace std;


BOOST_AUTO_TEST_CASE(BRIEFClusteringNN)
{
  //const static std::string pathToData("/home/straub/workspace_navvis/data/20120303_093952_Start0_Skip0_Reloc0_NFrames9999_Processed_20120407_094708");
//  const static std::string pathToData("/home/straub/workspace_navvis/data/20120420_175544_Start2_Skip0_Reloc0_NFrames9999_Processed_20120430_174916");
  const static std::string pathToData("/home/straub/workspace_navvis/data/20120420_175544_Start2_Skip0_Reloc0_NFrames9999_Processed_20120503_110414");

  std::vector<size_t> keyFrameNrs;
  std::vector<BriefData*> keyBriefs;
  std::map<BriefData*,size_t> keyBriefFrames;
  std::vector<BriefData*> briefs;
  std::map<BriefData*,size_t> briefFrames;

  if(!loadKeyFrameNrs(pathToData,keyFrameNrs)) return;
  if(!loadBriefs(pathToData,"allKeyBriefDescriptors.txt",keyBriefs, keyBriefFrames)) return;
  if(!loadBriefs(pathToData,"briefDescriptors.txt",briefs, briefFrames)) return;

//  {// some output so we can see that something was loaded
//    cout<<"-- loaded "<<keyBriefs.size()<<" briefs: "<<endl;
//    for(size_t i=0; i<keyBriefs.size(); ++i)
//      cout<<"In "<< briefFrames[keyBriefs[i]]<<"@"<<keyBriefs[i]->v2p[0]<<","<<keyBriefs[i]->v2p[1]<< ": "<<(*keyBriefs[i])<<endl;
//  }

  Random rnd(1);
  size_t M=150, N=keyBriefs.size(), Nall=briefs.size();

  vector<MapPoint*> queryMPs(M,NULL); // query MPs
  vector<MapPoint*> keyMPs(N,NULL); // MPs from  KeyFrames
  vector<MapPoint*> allMPs(Nall,NULL); // MPs from frames after KeyFrames
//  {// split into two sets for clustering
//    splitRandomly(rnd,keyBriefs,curMPs,allMPs);
//    cout<<"#curMPs="<<curMPs.size()<<" #mapMPs="<<allMPs.size()<<endl;
//  }

  {// connect BriefData to MapPoints and select random subset from allMps for query
    for(size_t i=0; i<N; ++i)
      keyMPs[i]=(&(keyBriefs[i]->pt));
    for(size_t i=0; i<Nall; ++i)
      allMPs[i]=(&(briefs[i]->pt));
    for(size_t i=0; i<queryMPs.size(); ++i)
      queryMPs[i]=allMPs[rnd(Nall)];
  }

//  vector<MapPoint*> curMPs;
//  vector<MapPoint*> allMPs;
//  {// split into two sets for clustering
//    size_t curFrame=(--briefFrames.end())->second -1;
//    BOOST_FOREACH(BriefData* brief, keyBriefs)
//    {
//      if(briefFrames[brief]==curFrame)
//      {
//        curMPs.push_back(&(brief->pt));
//      }else{
//        allMPs.push_back(&(brief->pt));
//      }
//    }
//    cout<<"#curMPs="<<curMPs.size()<<" #mapMPs="<<allMPs.size()<<endl;
//  }

  vector<Classifier*> cls;
  cls.push_back(new NearestNeighbor(keyMPs));
  cls.push_back(new LSH(keyMPs));
  cls.push_back(new kClusterClassifier<kBinaryMeans>(keyMPs));
  vector<pair<uint32_t,uint32_t> > clParams;
  clParams.push_back(pair<uint32_t,uint32_t>(0,0));
  clParams.push_back(pair<uint32_t,uint32_t>(10,32));
  clParams.push_back(pair<uint32_t,uint32_t>(50,0));

  for(uint32_t n=0; n<cls.size(); ++n)
  {
    vector<BriefAssociation> pairing;
    float dtPrep=cls[n]->prepare(clParams[n].first,clParams[n].second);
    float dtPair=cls[n]->pair(queryMPs,pairing);
    printPairing(keyBriefFrames,pairing);
    cout<< "========= dtPrep="<<dtPrep<<"ms dtPair="<<dtPair<<"ms ============"<<endl;
  }

  return;
  // ----------------------------------------- NN
  vector<BriefAssociation> pairingNN;
  Timer t0; t0.tic();
  NearestNeighbor NN(keyMPs); NN.pair(queryMPs,pairingNN);
  t0.toc();
  cout<<"--------- Time taken by NN: "<<t0<<endl;
  printPairing(keyBriefFrames,pairingNN);

  // ----------------------------------------- LSH
  vector<BriefAssociation> pairingLSH;
  t0.tic();
  LSH lsh(keyMPs); lsh.pair(queryMPs,pairingLSH);
  t0.toc();
  cout<<"--------- Time taken by LSH: "<<t0<<endl;
  printPairing(keyBriefFrames,pairingLSH);

  {// compare LSH pairing against NN (ground truth?)
    cout<<"compare LSH pairing against NN (ground truth?)"<<endl;
    uint32_t nEqual=0,nDiff=0;
    for(size_t i=0; i<pairingNN.size();++i)
      for(size_t j=0; j<pairingLSH.size();++j)
        if(pairingNN[i].mp1==pairingLSH[j].mp1)
        {
          if(pairingNN[i].mp2==pairingLSH[j].mp2)
          {
            ++nEqual;
          }else{
            ++nDiff;
          }
        }
    cout<<"#LSH matched equal to NN="<<nEqual<<" #LSH matched differently from NN="<<nDiff<< " (total#="<<pairingNN.size()<<")"<<endl;
  }

  // ----------------------------------------- kMedoids
  vector<BriefAssociation> pairingKMed;
  t0.tic();
  kClusterClassifier<kMedoids> kMed(keyMPs); kMed.pair(queryMPs,pairingKMed);
  t0.toc();
  cout<<"--------- Time taken by kMedoid: "<<t0<<endl;
  printPairing(keyBriefFrames,pairingKMed);

  {// empty memory
    for(size_t i=0; i<keyBriefs.size(); ++i)
    {
      delete &(keyBriefs[i]->pt);
      delete keyBriefs[i];
    }
  }
}

