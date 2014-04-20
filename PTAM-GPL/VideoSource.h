// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//
// VideoSource.h
// Declares the VideoSource class
// 
// This is a very simple class to provide video input; this can be
// replaced with whatever form of video input that is needed.  It
// should open the video input on construction, and provide two
// function calls after construction: Size() must return the video
// format as an ImageRef, and GetAndFillFrameBWandRGB should wait for
// a new frame and then overwrite the passed-as-reference images with
// GreyScale and Colour versions of the new frame.
//
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <vector>
#include <stdint.h>

struct VideoSourceData;

class VideoSource
{
 public:
  VideoSource();
  ~VideoSource();
  //void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
  /*
   * visodo-AM: extended method by imu data vectors. This information is only returned
   * by the ROS video source. As the parameters are addresses, the other video sources don't have
   * to return null, they just don't need to copy anything to those addresses
   */
  void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB,
		  std::vector<double> &vOrientation, std::vector<double> &vVelocity, std::vector<double> &vAcceleration,
		  int32_t &sec, int32_t &nsec, int32_t& seq);
  CVD::ImageRef Size();
  
 private:
  void *mptr;
  CVD::ImageRef mirSize;
};
