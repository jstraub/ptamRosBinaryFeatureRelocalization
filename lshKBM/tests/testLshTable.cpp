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

//#include "helpers.hpp"
//#include <MapPoint.h>
#include <BriefDesc.hpp>
#include <Random.hpp>
#include <lsh.hpp>
#include <timer.hpp>

#include <TooN/TooN.h>
#include <boost/test/unit_test.hpp>

#include <fstream>
#include <stdio.h>
#include <vector>
#include <map>
#include <time.h>


using namespace std;

// TODO: put into lsh.hpp (means adding a lsh.cpp)
Matrix<BriefDesc::N_BITS,BriefDesc::N_BITS> cov2corr(const Matrix<BriefDesc::N_BITS,BriefDesc::N_BITS>& cov)
{
  Matrix<BriefDesc::N_BITS,BriefDesc::N_BITS> corr;
  for(uint32_t i=0; i<BriefDesc::N_BITS; ++i)
    for(uint32_t j=0; j<BriefDesc::N_BITS; ++j)
      corr[i][j]=cov[i][j]/(sqrt(cov[i][i])*sqrt(cov[j][j]));
  return corr;
};


Matrix<BriefDesc::N_BITS,BriefDesc::N_BITS> getCovariance(string pathToCov)
{
  Matrix<BriefDesc::N_BITS,BriefDesc::N_BITS> cov;
  ifstream in(pathToCov.c_str());
  for(uint32_t i=0; i<BriefDesc::N_BITS; ++i)
    for(uint32_t j=0; j<BriefDesc::N_BITS; ++j)
      in>>cov[i][j];
  in.close();
  return cov;
};


Matrix<BriefDesc::N_BITS,BriefDesc::N_BITS> getCorrelation(string pathToCov)
{
  return cov2corr(getCovariance(pathToCov));
};


BOOST_AUTO_TEST_CASE(BRIEFClusteringLSH)
{
  // probability that a bit will change given that the BRIEF descriptors originate from a tracked feature
  // obtained using statisticsBRIEF
  const double bitChangeProbability[]={0.081680, 0.072040, 0.038572, 0.068813, 0.067742, 0.069282, 0.059201, 0.096044, 0.056842, 0.067365, 0.061110, 0.082050, 0.056219, 0.058355, 0.059604, 0.057742, 0.063037, 0.075570, 0.042593, 0.038219, 0.051036, 0.040769, 0.088124, 0.072708, 0.085566, 0.066525, 0.044762, 0.046201, 0.051766, 0.048676, 0.048639, 0.035992, 0.091912, 0.046403, 0.070990, 0.080334, 0.060774, 0.049791, 0.068506, 0.064217, 0.062835, 0.054091, 0.070879, 0.080894, 0.060107, 0.070044, 0.061301, 0.063895, 0.052727, 0.049941, 0.044068, 0.062183, 0.062601, 0.063433, 0.041991, 0.074226, 0.067918, 0.059715, 0.069335, 0.072862, 0.050870, 0.063906, 0.067840, 0.061976, 0.043504, 0.048745, 0.039579, 0.063486, 0.065294, 0.074580, 0.057033, 0.058943, 0.059800, 0.056364, 0.049192, 0.051577, 0.069835, 0.066641, 0.068577, 0.090795, 0.066821, 0.043746, 0.043305, 0.053635, 0.077135, 0.066246, 0.067707, 0.064930, 0.066049, 0.067089, 0.071007, 0.048247, 0.049977, 0.076395, 0.057506, 0.052870, 0.069148, 0.075331, 0.057037, 0.044870, 0.058168, 0.051825, 0.068949, 0.061821, 0.073417, 0.080762, 0.053933, 0.046585, 0.063659, 0.060841, 0.052280, 0.088878, 0.066844, 0.096845, 0.059348, 0.059730, 0.051253, 0.060179, 0.059122, 0.043241, 0.042362, 0.065317, 0.059963, 0.081554, 0.056210, 0.054292, 0.079112, 0.059821, 0.059669, 0.052604, 0.063187, 0.056118, 0.062678, 0.068403, 0.069538, 0.048654, 0.069602, 0.064221, 0.060267, 0.067573, 0.046767, 0.058457, 0.044082, 0.070815, 0.041179, 0.083143, 0.064448, 0.061667, 0.059562, 0.060706, 0.054735, 0.054339, 0.056532, 0.047240, 0.055583, 0.067887, 0.056501, 0.059372, 0.069281, 0.070621, 0.078277, 0.055576, 0.047427, 0.066214, 0.058596, 0.048022, 0.047606, 0.049718, 0.058454, 0.064349, 0.068399, 0.063577, 0.064120, 0.063776, 0.042621, 0.039238, 0.059761, 0.048475, 0.065815, 0.038063, 0.059788, 0.042917, 0.043040, 0.058410, 0.084213, 0.066698, 0.055562, 0.073858, 0.070393, 0.062040, 0.088566, 0.062163, 0.066347, 0.047092, 0.058857, 0.072002, 0.070717, 0.059469, 0.056009, 0.066361, 0.063113, 0.055520, 0.076284, 0.048267, 0.055808, 0.051873, 0.067395, 0.067061, 0.068243, 0.059593, 0.065634, 0.038977, 0.047671, 0.065772, 0.069570, 0.055426, 0.064971, 0.052142, 0.061817, 0.050353, 0.063160, 0.064857, 0.048640, 0.058869, 0.047176, 0.086396, 0.049126, 0.063849, 0.064076, 0.066677, 0.064603, 0.060489, 0.044680, 0.059285, 0.081641, 0.073160, 0.047258, 0.060129, 0.076182, 0.070560, 0.063611, 0.066236, 0.064797, 0.051942, 0.074020, 0.067812, 0.059891, 0.044904, 0.058378, 0.095531, 0.084141, 0.044783, 0.064936, 0.043507, 0.086203, 0.069262};
  vector<double> bitChangeProb(bitChangeProbability,bitChangeProbability+sizeof(bitChangeProbability) / sizeof(double) );

  Random rnd(time(NULL));
  uint32_t k=5;

  // ------------------------ threshold on the lowest 32 change probabilities - everything else is assumed to change all the time
  vector<double> bitChangeProbSorted(bitChangeProb);
  std::sort(bitChangeProbSorted.begin(),bitChangeProbSorted.end());

  for(uint32_t i=0; i<bitChangeProb.size(); ++i)
    if(bitChangeProb[i]>bitChangeProbSorted[32])
      bitChangeProb[i]=1;
  for(uint32_t i=0; i<bitChangeProb.size(); ++i)
      cout<<bitChangeProb[i]<<" ";
  cout<<endl;

  LshTable<BriefDesc> lshT0(rnd,LshParams(1,k),bitChangeProb);
  vector<uint32_t> g0=lshT0.getHashFunction();
  cout<<"Hash function "<<k<<" :"<<endl;
  BOOST_FOREACH(uint32_t gi, g0)
  {
    cout<<gi<<" ";
  }; cout<<endl;

  // ------------------------ deterministic changes
  for(uint32_t i=0; i<bitChangeProb.size(); ++i)
    bitChangeProb[i]=1;
  for(uint32_t i=0; i<10; ++i)
    bitChangeProb[i]=0;
  for(uint32_t i=0; i<bitChangeProb.size(); ++i)
    cout<<bitChangeProb[i]<<" ";
  cout<<endl;

  LshTable<BriefDesc> lshT1(rnd,LshParams(1,k),bitChangeProb);
  vector<uint32_t> g1=lshT1.getHashFunction();
  cout<<"Hash function "<<k<<" :"<<endl;
  BOOST_FOREACH(uint32_t gi, g1)
  {
    cout<<gi<<" ";
  }; cout<<endl;

  // ------------------------ without probability prior
  LshTable<BriefDesc> lshT2(rnd,LshParams(1,k));
  vector<uint32_t> g2=lshT2.getHashFunction();
  cout<<"Hash function "<<k<<" :"<<endl;
  BOOST_FOREACH(uint32_t gi, g2)
  {
    cout<<gi<<" ";
  }; cout<<endl;

  // ----------------------- correlation
  cout<<"Correlation"<<endl;
  Matrix<BriefDesc::N_BITS,BriefDesc::N_BITS> cov=getCovariance("covBriefsAll.txt");
  Matrix<BriefDesc::N_BITS,BriefDesc::N_BITS> corr=cov2corr(cov); // should be exactly the same since diagonal elements are 1

  for(uint32_t i=0; i<BriefDesc::N_BITS; ++i){
    for(uint32_t j=0; j<BriefDesc::N_BITS; ++j){
      char buff[8];
      sprintf(buff,"%4.2lf ",cov[i][j]);
      cout<<buff;
    }
    cout<<";"<<endl;
  }

  for(uint32_t i=0; i<BriefDesc::N_BITS; ++i){
      for(uint32_t j=0; j<BriefDesc::N_BITS; ++j){
        char buff[8];
        sprintf(buff,"%4.2lf ",corr[i][j]);
        cout<<buff;
      }
      cout<<";"<<endl;
  }
  k=20;
  vector<uint32_t> gs;
  cout<<"Hash functions "<<k<<" :"<<endl;
  for(uint32_t i=0; i<60; ++i)
  {
    LshTable<BriefDesc> lshT3(rnd,LshParams(1,k,1,1),&corr);
    vector<uint32_t> g3=lshT3.getHashFunction();
    BOOST_FOREACH(uint32_t gi, g3)
    {
      cout<<gi<<"\t";
      gs.push_back(gi);
    };
    cout<<endl;
  }

  sort(gs.begin(),gs.end());
  for(uint32_t i=0; i<gs.size(); ++i)
  {
    cout<<gs[i]<<" ";
  }
  cout<<endl;

  //start matlab for plotting the results
  //system("matlab -nosplash -nodesktop -r \"addpath /home/straub/workspace_navvis/matlab/; plotLSHResults\";");
  //system("stty echo;");

  cout<<"Exiting"<<endl;
}


