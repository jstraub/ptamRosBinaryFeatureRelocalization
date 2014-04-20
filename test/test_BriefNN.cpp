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
#include "helpers.hpp"
#include <MapPoint.h>
#include <BriefData.hpp>
#include <Random.hpp>
#include <nn.hpp>

#include <timer.hpp>

#include <fstream>
#include <stdio.h>
#include <vector>
#include <map>


using namespace std;

void splitRandomly(Random& rnd, std::vector<BriefData*>& keyBriefs,
    vector<BriefData*>& curMPs, vector<BriefData*>& allMPs)
{// split into two sets for clustering
  for(size_t i=0; i<keyBriefs.size(); ++i)
    allMPs[i]=keyBriefs[i];
  for(size_t i=0; i<curMPs.size(); ++i)
    curMPs[i]=allMPs[rnd(keyBriefs.size())];
}

void printPairing(std::map<BriefData*,size_t>& frame,
    vector<Assoc<BriefData,uint32_t> >& pairing)
{// show classification frame Nrs
  cout<<"Classification frame<->frame via features"<<endl;
  for(size_t i=0; i<pairing.size();++i)
    if(pairing[i].m!=NULL)
    {
      cout<<pairing[i].d <<": "<<frame[pairing[i].q]<<"<->"
          <<frame[pairing[i].m]<< "\t";
    }
}

BOOST_AUTO_TEST_CASE(BRIEFClusteringNN)
{
  //const static std::string pathToData("/home/straub/workspace_navvis/data/20120303_093952_Start0_Skip0_Reloc0_NFrames9999_Processed_20120407_094708");
  const static std::string pathToData("/home/straub/workspace_navvis/data/20120420_175544_Start2_Skip0_Reloc0_NFrames9999_Processed_20120430_174916");

  std::vector<size_t> keyFrameNrs;
  std::vector<BriefData*> keyBriefs;
  std::map<BriefData*,size_t> briefFrames;

  if(!loadKeyFrameNrs(pathToData,keyFrameNrs)) return;
  if(!loadBriefs(pathToData,"allKeyBriefDescriptors.txt",keyBriefs, briefFrames)) return;

  {// some output so we can see that something was loaded
    cout<<"-- loaded "<<keyBriefs.size()<<" briefs: "<<endl;
    for(size_t i=0; i<keyBriefs.size(); ++i)
      cout<<"In "<< briefFrames[keyBriefs[i]]<<"@"<<keyBriefs[i]->v2p[0]<<","<<keyBriefs[i]->v2p[1]<< ": "<<(*keyBriefs[i])<<endl;
  }

  Random rnd(1);
  size_t M=150, N=keyBriefs.size();

  vector<BriefData*> curMPs(M,NULL);
  vector<BriefData*> allMPs(N,NULL);
  {// split into two sets for clustering
    splitRandomly(rnd,keyBriefs,curMPs,allMPs);
    cout<<"#curMPs="<<curMPs.size()<<" #mapMPs="<<allMPs.size()<<endl;
  }

  // ----------------------------------------- NN
  vector<Assoc<BriefData,uint32_t> > pairingNN;
  Timer t0; t0.tic();
  NearestNeighbor<BriefData,uint32_t> NN(allMPs);
  dynamic_cast<Classifier<BriefData,uint32_t>* >(&NN)->pair(curMPs,pairingNN);
  t0.toc();
  cout<<"--------- Time taken by NN: "<<t0<<endl;
  printPairing(briefFrames,pairingNN);

  {// empty memory
    for(size_t i=0; i<keyBriefs.size(); ++i)
    {
      delete &(keyBriefs[i]->pt);
      delete keyBriefs[i];
    }
  }
}

