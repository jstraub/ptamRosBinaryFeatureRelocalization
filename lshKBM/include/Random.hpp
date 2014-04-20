/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef RANDOM_HPP_
#define RANDOM_HPP_

//#include <boost/random/random_number_generator.hpp>
//#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>


#include <stdint.h>
#include <vector>
#include <map>

using namespace std;
//#define NEW_BOOST

#ifdef NEW_BOOST

class Random
{
public:

  boost::mt19937 mGen;
  boost::random::uniform_int_distribution<uint32_t> mUnif;
  boost::variate_generator< boost::mt19937, boost::random::uniform_int_distribution<uint32_t> > mRand;
  Random(unsigned long seed):// call instance:
    mGen(seed), mUnif(), mRand(mGen, mUnif) {
  }
//  std::ptrdiff_t operator()( std::ptrdiff_t arg ) {
//    return static_cast< std::ptrdiff_t >( mRand()%arg );
//  };

  uint32_t operator()( uint32_t  arg ) {
      return static_cast<uint32_t>( mRand()%arg );
    };

};

#else

class Random
{
public:
  boost::mt19937 mGen;
  boost::uniform_int<uint32_t> mUnif;
  boost::variate_generator< boost::mt19937, boost::uniform_int<uint32_t> > mRand;
  Random(unsigned long seed):// call instance:
    mGen(seed), mUnif(0,(std::numeric_limits<uint32_t>::max)()), mRand(mGen, mUnif) {
  }
//  std::ptrdiff_t operator()( std::ptrdiff_t arg ) {
//    return static_cast< std::ptrdiff_t >( mRand()%arg );
//  };

  void resetRepetitions()
  {
    if(!drawn.empty()) drawn.clear();
  };

  uint32_t drawWithoutRepetition(uint32_t arg, bool reset=false)
  {
    if(reset) resetRepetitions();
    uint32_t rndNr=(*this)(arg);
    while(drawn.find(rndNr)!=drawn.end()) rndNr=(*this)(arg);
    drawn[rndNr]=true; // insert something so find will have a entry next time
    return rndNr;
  }

  uint32_t operator()( uint32_t  arg ) {
      return static_cast<uint32_t>( mRand()%arg );
    };

private:
  map<uint32_t,bool> drawn;
};

#endif /* NEW_BOOST */

#endif /* RANDOM_HPP_ */
