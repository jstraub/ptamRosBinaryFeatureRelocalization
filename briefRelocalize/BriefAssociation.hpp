/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef BRIEFASSOCIATION_HPP_
#define BRIEFASSOCIATION_HPP_

#include "MapPoint.h"

#include <float.h>
#include <limits.h>

using namespace std;

struct BriefAssociation
{
  BriefAssociation(uint32_t d=INT_MAX, MapPoint *p1=NULL,
      MapPoint * p2=NULL)//, uint32_t mp2I=0)
    : dist(d),mp1(p1),mp2(p2)//,mp2Id(mp2I)
  {};

  uint32_t dist;
  MapPoint *mp1; // usually the query
  MapPoint *mp2; // usually the match
//  uint32_t mp2Id; // id of match in candidate set

  bool operator< (const BriefAssociation& b) const{
    return (dist<b.dist);
  }
};
// association


#endif /* BRIEFASSOCIATION_HPP_ */
