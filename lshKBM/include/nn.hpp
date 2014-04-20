/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef NN_HPP_
#define NN_HPP_

#include <classifier.hpp>
#include <boost/foreach.hpp>
#include <association.hpp>

using namespace std;

template<class Desc, class Dist>
class NearestNeighbor : public Classifier<Desc,Dist>
{
public:
  NearestNeighbor(const vector<Desc*>& mapDs, Params p_=Params())
    : Classifier<Desc,Dist>(mapDs,p_)
  {};
  NearestNeighbor(const NearestNeighbor& nn)
    : Classifier<Desc,Dist>(nn.getMapDs(),nn.getParams())
  {};

  float prepare(void) // no preparation necessary
  {return 0.0;};

  // do NN pairing of one querypoint
  // the results of this query are simply appended to briefPair
  bool pair(Desc* queryD, vector<Assoc<Desc,Dist> >& briefPair)
  {
    Dist distMin=UnitExtreme<Dist>::max();
    vector<Desc*> closestDs(1,NULL);
    for(uint32_t i=0; i<this->mMapDs.size(); ++i)
    {
      assert(this->mMapDs[i]!=NULL);
      uint32_t dist=queryD->dist(this->mMapDs[i]);
      if(distMin > dist)
      {
        distMin=dist;
        closestDs[0]=this->mMapDs[i];
        closestDs.resize(1,NULL);
      }else if(distMin == dist)
        closestDs.push_back(this->mMapDs[i]);
    }
    assert(distMin<257);
    briefPair.reserve(briefPair.size()+closestDs.size());
    for(uint32_t i=0; i<closestDs.size(); ++i)
      briefPair.push_back(Assoc<Desc,Dist>(queryD,closestDs[i],distMin));

//    cout<<closestDs.size()<<" ";
    return true;
  };
  bool pair(Desc* queryMP, vector<Assoc<Desc,Dist> >& briefPair, uint32_t& retrieved)
  {
    retrieved=1;
    return pair(queryMP, briefPair);
  };

  /* do kNN pairing of own query point
   *
   * @param pairing: pairing is resized to have size of mMapDs and sorted to have the first
   *                 k pairings in the front. Pairing is NOT resized to have only k elements
   *                 to do less memory reallocations
   */
  void kPair(Desc* queryD, vector<Assoc<Desc,Dist> >& pairing, uint32_t k)
  {
    pairing.resize(this->mMapDs.size(),Assoc<Desc,Dist>());
    for(uint32_t i=0; i<this->mMapDs.size(); ++i)
    {
      assert(this->mMapDs[i]!=NULL);
      pairing[i]=Assoc<Desc,Dist>(queryD,this->mMapDs[i],queryD->dist(this->mMapDs[i]));
    }
    // sort only the first k elements -> the rest is unsorted!
    if(pairing.size() <= k)
      sort(pairing.begin(), pairing.end());
    else{
      partial_sort(pairing.begin(), pairing.begin()+k, pairing.end());
    }
  };
  void kPair(Desc* queryD, vector<Assoc<Desc,Dist> >& pairing, uint32_t& nRetrieved, uint32_t k)
  {
    nRetrieved=1;
    kPair(queryD, pairing, k);
  };

  bool pair(Desc* queryD, Assoc<Desc,Dist>& annPair, uint32_t& nRetrieved, uint32_t& support)
  {
//    cout<<"eval support nn"<<endl;
    support=1;
    vector<Assoc<Desc,Dist> > annPairs;
    bool ret=pair(queryD,annPairs,nRetrieved);
    annPair=annPairs[0];
    return ret;
  };
};



#endif /* NN_HPP_ */
