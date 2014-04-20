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

#include <Random.hpp>

#include <stdio.h>
#include <iostream>
#include <time.h>

using namespace std;

BOOST_AUTO_TEST_CASE(testRandom)
{
  Random rnd(time(NULL));
  uint32_t N=1000;
  cout<<"Random numbers between 0 and "<<N<<endl;
  for(uint32_t i=0; i<100; ++i)
  {
    cout<<rnd(N)<<" ";
  }
  cout<<endl;

  char buff[10]; sprintf(buff,"%6.2lf",double(0.0));
  cout<<"Progress: "<<string(buff); cout.flush();
  usleep(2000000);
  sprintf(buff,"%6.2lf",double(4.39));
  cout<<"\b\b\b\b\b\b"<<string(buff); cout.flush();
  usleep(2000000);
  sprintf(buff,"%6.2lf",double(45.39));
  cout<<"\b\b\b\b\b\b"<<string(buff); cout.flush();
}



