/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 

#ifndef OUTPUTHELPER_HPP_
#define OUTPUTHELPER_HPP_

#include "KeyFrame.h"

#include <TooN/se3.h>
#include <boost/foreach.hpp>

//#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <sstream>

class VectorToFile
{
public:
  VectorToFile(const char* pathToFile): mPathToFile(pathToFile), mO(pathToFile)
  {};

  ~VectorToFile()
  {
    mO.flush();
    mO.close();
  };

  std::ofstream& out(void)
  {
    return mO;
  }

  // incremental output the content of a vector of values using operator<<
  template<class T>
  void write(const T& val)
  {
    mO<<val<<std::endl;
  };
  template<class T1,class T2>
  void write(const T1& val1,const T2& val2)
  {
    mO<<val1<<" "<<val2<<std::endl;
  };
  template<class T1,class T2, class T3>
  void write(const T1& val1,const T2& val2,const T3& val3 )
  {
    mO<<val1<<" "<<val2<<" "<<val3<<std::endl;
  };


  template<class T>
  void write(const T& val, std::string(*toString)(T))
  {
    mO<<(*toString)(val);
  };

  template<class T>
  void writeBatch(const std::vector<T>& vals){
    BOOST_FOREACH(T val, vals)
    {
      mO<<val<<std::endl;
    }
  };

  enum valueType{first,second};

  // output map members. Tout specifies whether we output T1 or T2.
  template<class T1,class T2>
  void writeBatch(const std::map<T1,T2>& vals, valueType choice)
  {
    if(choice==first)
    {
      for(typename std::map<T1,T2>::const_iterator it=vals.begin(); it!=vals.end(); ++it)
      {
        mO<<it->first<<std::endl;
      }
    }else{
      for(typename std::map<T1,T2>::const_iterator it=vals.begin(); it!=vals.end(); ++it)
      {
        mO<<it->second<<std::endl;
      }
    }
  };

  template<class T>
  void writeBatch(const std::vector<T>& vals, std::string(*toString)(T)){
    BOOST_FOREACH(T val, vals)
    {
      mO<<(*toString)(val)<<std::endl;
    }
  };

  // output the content of a vector of values using operator<<
  template <class T>
  static void vectorToFile(const std::vector<T>& vals, char* pathToFile);
  // output the content of a vector of values using a output function supplied via toString(T).
  template <class T>
  static void vectorToFile(const std::vector<T>& vals, char* pathToFile, std::string (*toString)(T));

private:
  const char* mPathToFile;
  std::ofstream mO;
};

std::ostream& operator<<(std::ostream& out, const Measurement& meas);
std::ostream& operator<<(std::ostream& out, const cv::KeyPoint& kp);

// converts SE3 into a serial string of the translation and rotation information
// output in column-major - for matlab! just use reshape(x,3,4)
std::string serializeForOutput(TooN::SE3<> pose);


#endif /* OUTPUTHELPER_HPP_ */
