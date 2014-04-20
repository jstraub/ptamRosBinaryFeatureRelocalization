/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef RESULTS_HPP_
#define RESULTS_HPP_

#include <classifier.hpp>

#include <TooN/TooN.h>
#include <TooN/se3.h>

#include <stdint.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <assert.h>

using namespace std;
using namespace TooN;


struct LocResult : public ClassifierResult
{
  LocResult() : ClassifierResult()
  {};

  float tExtract; // feature extraction
  float tRANSAC; // RANSAC
  float tMEst; // MEst
  float tReloc; // time for relocalisation

  SE3<double> wTc;
  uint32_t inliers; // how many inliers does this wTc have
  vector<Vector<3,double> > p3d; // 3D points that were used to localize

  void dump() const
  {
    cout<<"wTc:"<<endl<<wTc;
    cout<<"Has "<<inliers<<" inliers from "<<querySize
        <<" queried in "<<tPair<<"ms"<<endl;
  };
};

string serializeForOutput(SE3<double> pose);
long int writeRelocResults(const vector<LocResult>& locRes, const vector<uint32_t>& frameIDs,
    string dir, long int timeTag=0, string tag="noTag");


#endif /* RESULTS_HPP_ */
