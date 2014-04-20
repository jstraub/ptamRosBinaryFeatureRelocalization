/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef BRIEFEXTRACTOR_HPP_
#define BRIEFEXTRACTOR_HPP_

#include "BriefData.hpp"
#include "MapPoint.h"
#include "TrackerData.h"
#include "KeyFrame.h"
#include "outputHelper.hpp"
#include "timer.hpp"

#include <cvd/image_ref.h>
#include <cvd/image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/foreach.hpp>
#include <vector>

using namespace std;
using namespace CVD;

struct MapPoint;

class BinaryExtractor
{
public:

  BinaryExtractor(enum Feature_t featureType=BRIEF_T_HIST): mFeatureType(featureType)
  {
    assert(mFeatureType<N_TYPES);
  };

  ~BinaryExtractor()
  {
    BOOST_FOREACH(MapPoint* mp, mPFastCorner)
    {
      delete mp->pBData;
      delete mp->pTData;
      delete mp;
    }
  };

  void extractForMapPoints(KeyFrame& kF, int32_t frameNr) const;
  //not static because this class stores pointers to MapPoints in order to delete them after usage.
  void extractForFastCorners(KeyFrame &kF, vector<MapPoint*>& mps);

  void saveDescriptors(ofstream& outDesc, ofstream& outStat, KeyFrame& kF, int32_t frameNr);

  static void image2cvMat(Image<byte>& im, cv::Mat& img)
  {
    img=cv::Mat(im.size().y,im.size().x,CV_8U);
    img.data=im.data();
//    cout<<"Copied CvD Image(keyframe) to cv::Mat "<<im.size().y<<"x"<<im.size().x<<"->"<<img.rows<<"x"<<img.cols<<endl;
  }

private:

  Feature_t mFeatureType;
  vector<MapPoint*> mPFastCorner;

  // converts the meas to KeyPoints kps (clears kps before conversion); kps is in a single pyramid level specified by octave
  static void convertToCvKeyPoints(vector<Candidate>& meas ,vector<cv::KeyPoint>& kps, int octave,
      bool append=false);
  // converts the meas to KeyPoints kps (clears kps before conversion); kps stores a vector of Keypoints per pyramid level;
  // corresp stores the association from KeyPoint to MapPoint*
  static void convertToCvKeyPoints(map<MapPoint*, Measurement>& meas, vector<vector<cv::KeyPoint> >& kps, vector<MapPoint*>& corresp);
  // same as method above but does not sort keypoints into different bins according to pyramid level
  static void convertToCvKeyPoints(map<MapPoint*, Measurement>& meas, vector<cv::KeyPoint>& kps, vector<MapPoint*>& corresp);

  // does the extraction of BRIEF feature descriptors from given KeyPoints in the
  // given image
  void extractBRIEF(vector<cv::KeyPoint>& kps, const cv::Mat& img, cv::Mat& bDesc) const;
  void extractBRIEF(vector<cv::KeyPoint>& kps, Image<byte>& im, cv::Mat& bDesc) const
  {
    cv::Mat img;
    image2cvMat(im,img);
    extractBRIEF(kps,img,bDesc);
  }
};



#endif /* BRIEFEXTRACTOR_HPP_ */
