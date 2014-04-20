/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 
#include "outputHelper.hpp"

template <class T>
void VectorToFile::vectorToFile(const std::vector<T>& vals, char* pathToFile)
{
  std::ofstream o(pathToFile);
  BOOST_FOREACH(T val, vals)
  {
    o<<val<<std::endl;
  }
  o.flush();
  o.close();
}

template <class T>
void VectorToFile::vectorToFile(const std::vector<T>& vals, char* pathToFile, std::string (*toString)(T))
{
  if(toString==NULL) return;

  std::ofstream o(pathToFile);
  BOOST_FOREACH(T val, vals)
  {
    o<<(*toString)(val)<<std::endl;
  }
  o.flush();
  o.close();
}


std::ostream& operator<<(std::ostream& out, const Measurement& meas)
{
  out<<meas.v2RootPos[0] << "\t"<< meas.v2RootPos[1] << "\t" << meas.nLevel;
  return out;
}

std::ostream& operator<<(std::ostream& out, const cv::KeyPoint& kp)
{
  out<<"("<< kp.pt.x << ","<< kp.pt.y << ") in" << kp.octave;
  return out;
}

std::string serializeForOutput(TooN::SE3<> pose)
{
  std::stringstream so;
  TooN::Vector<3>& trans=pose.get_translation();
  TooN::Matrix<3, 3> rot=pose.get_rotation().get_matrix(); // row major
  // output in column-major - for matlab! just use reshape(x,3,4)
  for (size_t j = 0; j < 3; ++j)
    for (size_t i = 0; i < 3; ++i)
      so << rot(i, j) << " ";
  for (size_t i = 0; i < 3; ++i)
      so << trans[i] << " ";
  return so.str();
}
