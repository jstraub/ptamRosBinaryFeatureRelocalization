/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #include "BriefRelocalizer.hpp"


vector<BriefData*>& mapPoint2BriefData(const vector<MapPoint*>& mapMPs, vector<BriefData*>& mapBDs)
{
  mapBDs.reserve(mapMPs.size());
  for(uint32_t i=0; i< mapMPs.size(); ++i)
    mapBDs.push_back(mapMPs[i]->pBData);
  return mapBDs;
}

vector<BriefData*> mapPoint2BriefData(const vector<MapPoint*>& mapMPs)
{
  vector<BriefData*> mapBDs;
  mapPoint2BriefData(mapMPs, mapBDs);
  return mapBDs;
}
