/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 

#ifndef ASSOCIATION_HPP_
#define ASSOCIATION_HPP_

#include <stdint.h>
#include <float.h>
#include <climits>
#include <stddef.h>

using namespace std;

template<class Unit>
class UnitExtreme{
public:
  virtual ~UnitExtreme()
  {};
  static Unit max(){return mMax;};
  static Unit min(){return mMin;};
protected:
  static const Unit mMax;
  static const Unit mMin;
};
template<>
class UnitExtreme<double>
{
public:
  UnitExtreme()
  {};
  static double max(){return mMax;};
  static double min(){return mMin;};
protected:
  static const double mMax=FLT_MAX;
  static const double mMin=-FLT_MAX;
};
template<>
class UnitExtreme<uint32_t>
{
public:
  UnitExtreme()
  {};
  static double max(){return mMax;};
  static double min(){return mMin;};
protected:
  static const uint32_t mMax=INT_MAX;
  static const uint32_t mMin=0;
};

/* Association between two descriptors with a distance
 *
 */
template<class Desc, class Dist>
struct Assoc
{
  Assoc(Desc* query=NULL, Desc* match=NULL, Dist dist=UnitExtreme<Dist>::max())
    : q(query), m(match), d(dist)
  {};

  Assoc(const Assoc<Desc,Dist>& a)
    : q(a.q), m(a.m), d(a.d)
  {};

  Desc* q; // query
  Desc* m; // match
  Dist d; // distance

  bool operator< (const Assoc<Desc,Dist>& b) const
  {
    return (d<b.d);
  }
  Assoc<Desc,Dist>& operator= (const Assoc<Desc,Dist>& b)
  {
    if(this!=&b){
      //assign values
      q=b.q;
      m=b.m;
      d=b.d;
    }
    return *this;
  };
};
//typedef Assoc<BriefDesc,uint32_t> BriefAssoc;

#endif /* ASSOCIATION_HPP_ */
