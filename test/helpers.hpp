/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef HELPERS_HPP_
#define HELPERS_HPP_

#include <MapPoint.h>
#include <BriefData.hpp>
#include <BriefAssociation.hpp>
#include <Random.hpp>

#include <fstream>
#include <stdio.h>
#include <vector>
#include <map>

using namespace std;

void splitRandomly(Random& rnd, std::vector<BriefData*>& keyBriefs,
    vector<MapPoint*>& curMPs, vector<MapPoint*>& allMPs);

void printPairing(std::map<BriefData*,size_t>& frame,
    vector<BriefAssociation>& pairing);

void compairAgainstNN(const vector<BriefAssociation>& pairing,
    const vector<BriefAssociation>& pairingNN,
    uint32_t& tp);

void evalPairing(const vector<BriefAssociation>& pairing, double& avgDist,
    uint32_t& notPaired);

//void getBRIEFchange(vector<MapPoint*>& mps,
//    vector<uint32_t>& change, uint32_t& N);
//
//void getBRIEFStatistics(vector<MapPoint*>& mps, vector<double>& mean,
//    vector<double>& std, vector<double>& entropy);
//
//double getMeanIntroClusterDist(vector<MapPoint*>& mps);

//bool loadPoses(const std::string pathToData,
//    const std::string fileName,
//    std::map<uint32_t,SE3<>* >& poses);

#endif /* HELPERS_HPP_ */
