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
#include <timer.hpp>

#include <stdio.h>

using namespace std;

BOOST_AUTO_TEST_CASE(BRIEFfromArrayAndDistance)
{
  MapPoint mP1,mP2;
  BriefDataS *bd1=new BriefDataS(&mP1);
  BriefDataS *bd2=new BriefDataS(&mP2);

  mP1.pBData=bd1;
  mP2.pBData=bd2;

  // check pointers and references
  BOOST_CHECK_EQUAL(mP1.pBData,bd1);
  BOOST_CHECK_EQUAL(mP2.pBData,bd2);
  BOOST_CHECK_EQUAL(&mP1,&bd1->pt);
  BOOST_CHECK_EQUAL(&mP2,&bd2->pt);

  // check BRIEF descriptor generation form array and distance measurement.
  uint8_t d1[BriefData::BRIEF_K],d2[BriefData::BRIEF_K];
  for(uint32_t i=0;i<BriefData::BRIEF_K;++i){d1[i]=0; d2[i]=0;}
  bd1->descriptorFromCvMatRow(d1); for(uint32_t i=0;i<BriefData::BRIEF_K;++i){BOOST_CHECK_EQUAL(bd1->bd[i],0);}
  bd2->descriptorFromCvMatRow(d2); for(uint32_t i=0;i<BriefData::BRIEF_K;++i){BOOST_CHECK_EQUAL(bd2->bd[i],0);}
  BOOST_CHECK_EQUAL(bd1->dist(*bd2),bd2->dist(*bd1));

  d2[0]=1; bd2->descriptorFromCvMatRow(d2);
  {
    BOOST_CHECK_EQUAL(bd2->bd[0],1);
    for(uint32_t i=1;i<BriefData::BRIEF_K;++i){BOOST_CHECK_EQUAL(bd2->bd[i],0);}
  }
  BOOST_CHECK_EQUAL(bd1->dist(*bd2),1.0);

  d2[1]=1; bd2->descriptorFromCvMatRow(d2);
  {
    BOOST_CHECK_EQUAL(bd2->bd[0],1);
    BOOST_CHECK_EQUAL(bd2->bd[1],1);
    for(uint32_t i=2;i<BriefData::BRIEF_K;++i){BOOST_CHECK_EQUAL(bd2->bd[i],0);}
  }
  BOOST_CHECK_EQUAL(bd1->dist(*bd2),2.0);

  d2[5]=1; bd2->descriptorFromCvMatRow(d2);
  BOOST_CHECK_EQUAL(bd1->dist(*bd2),3.0);

  // check BRIEF descriptor generation from score
  std::vector<uint32_t> score1(BriefData::BRIEF_K*8,10);
  bd1->descriptorFromBriefScore(score1,100); for(uint32_t i=0;i<BriefData::BRIEF_K;++i){BOOST_CHECK_EQUAL(bd1->bd[i],uint8_t(0));}
  std::vector<uint32_t> score2(BriefData::BRIEF_K*8,75);
  bd2->descriptorFromBriefScore(score2,100); for(uint32_t i=0;i<BriefData::BRIEF_K;++i){BOOST_CHECK_EQUAL(bd2->bd[i],uint8_t(255));}

  cout<<" ---- timing comparison descriptorFromScore ---- "<<endl;
  std::vector<uint32_t> score3(BriefData::BRIEF_K*8,75);
  for(uint32_t i=0;i<BriefData::BRIEF_K*4;++i){score3[i]=10;};
  uint32_t N=1000000;
  Timer t0;
  for(uint32_t i=0; i<N; ++i)
    bd2->descriptorFromBriefScore(score3,100);
  t0.toc();
  cout<<" descriptorFromBriefScoreMult:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;
  t0.tic();
  for(uint32_t i=0; i<N; ++i)
    bd2->descriptorFromBriefScoreDiv(score3,100);
  t0.toc();
  cout<<" descriptorFromBriefScoreDiv:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;


  cout<<" ---- timing comparison dist ---- "<<endl;
  N=100000000;
  t0.tic();
  for(uint32_t i=0; i<N; ++i)
    dynamic_cast<BriefDesc*>(bd2)->distIterative(*bd1);
  t0.toc();
  cout<<" iterative distance:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;
  t0.tic();
  for(uint32_t i=0; i<N; ++i)
    bd2->dist(bd1);
  t0.toc();
  cout<<" 8bit LUT distance:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;
  t0.tic();
  for(uint32_t i=0; i<N; ++i)
    bd2->dist16bit(*bd1);
  t0.toc();
  cout<<" 16bit LUT distance:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;


  delete bd1;
  delete bd2;
}
