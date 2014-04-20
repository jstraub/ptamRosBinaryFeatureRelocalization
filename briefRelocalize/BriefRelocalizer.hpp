/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef BRIEFRELOCALIZER_HPP_
#define BRIEFRELOCALIZER_HPP_

#include "MapPoint.h"
#include "TrackerData.h"
#include "Map.h"
#include "BriefExtractor.hpp"
#include "MEstimator.h"
#include "Relocaliser.h"
#include "Random.hpp"
#include "Ransac.hpp"
#include "ATANCamera.h"
#include <lsh.hpp>
#include <TooN/wls.h>
#include <TooN/se2.h>
#include <cvd/image_io.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>
#include <float.h>
#include <stdio.h>
#include <fstream>

struct TrackerData;

//#define OUTPUT_RELOCALIZATION_FILES
using namespace CVD;
using namespace std;
using namespace GVars3;

vector<BriefData*>& mapPoint2BriefData(const vector<MapPoint*>& mapMPs, vector<BriefData*>& mapBDs);
vector<BriefData*> mapPoint2BriefData(const vector<MapPoint*>& mapMPs);


/* Wrapper of LSH for PTAM
 *
 */
//class LSHptam : public LSH<BriefData,uint32_t>
//{
//public:
//  LSHptam(const vector<MapPoint*>& mapMPs, const Params& lshParams, double filteringRadius=5.0)
//  : LSH<BriefData,uint32_t>(mapPoint2BriefData(mapMPs, mMapBDs),lshParams), mMapMPs(mapMPs),
//    mFilteringRadius(filteringRadius), doFiltering(false)
//  {};
//  ~LSHptam()
//  {};
//
//  float prepare()
//  {
//    // update the mMapBDs from the reference to the map points in the map object of PTAM.
////    mapPoint2BriefData(mMapMPs, mMapBDs);
//    for(uint32_t i=0; i< mMapMPs.size(); ++i)
//    {
//      // go from cTw to wTc in order to get translation of camera
//      Vector<3,double> pos=mFiltPose.inverse().get_translation();
//      if((doFiltering && norm(pos - mMapMPs[i]->v3WorldPos) < mFilteringRadius) || !doFiltering)
//        for(uint32_t j=0; j<mMapMPs[i]->bData.size(); ++j)
//          mMapBDs.push_back(mMapMPs[i]->bData[j]);
//    }
//    cout<<"Preparing "<<mMapBDs.size()<<" map points for LSH retrieval"<<endl;
//    return LSH<BriefData,uint32_t>::prepare();
//  };
//
//  void setFiltering(const SE3<double>& filtPose, double filteringRadius=5.0)
//  {
//    mFiltPose=filtPose;
//    mFilteringRadius=filteringRadius;
//  }
//  void turnOnFilteing()
//  {
//    doFiltering=true;
//  }
//  void turnOffFilteing()
//  {
//    doFiltering=false;
//  }
//
//private:
//
//  float prepare(void);
//
//  const vector<MapPoint*>& mMapMPs;
//  vector<BriefData*> mMapBDs;
//
//  SE3<double> mFiltPose; // last pose used for filtering
//  double mFilteringRadius;
//  bool doFiltering;
//};

// Localizer based on brief features
//
// BRIEFRelocalizer class needs to:
//   ‣ Same interface as existing ZMSSD Relocalizer
// • Relocaliser(Map &map, ATANCamera &camera);
// • bool AttemptRecovery(KeyFrame &k); // KeyFrame here is the current KeyFrame
// • SE3<> BestPose();
//   ‣ Find MapPoints that are L2 close and within a certain solid angle (Raumwinkel).
//     For this set compute the BRIEF descriptors (if not yet stored).
//   ‣ Compute a pose estimate for the camea from the detected associations
template<class RANSAC, class Mod, class Desc>
class BriefRansacRelocaliser: public Relocaliser
{
public:
  BriefRansacRelocaliser(Mod* model,
      LSH<BriefData,uint32_t>* classifier,
      const RansacParams& ransacParams, double width=640, double height=480)
  : Relocaliser(model->getCamModel()),
    mpClassifier(classifier),
    mRnd(time(NULL)),
    mpModel(model), mRansac(mRnd,*mpModel,ransacParams),
    imgWidth(width), imgHeight(height)
  {};

  ~BriefRansacRelocaliser()
  {
    delete mpModel;
    delete mpClassifier;
  };

  bool AttemptRecovery(vector<Desc*>& queryDs, SE3<double>& lastPose, double maxDist=5.0)
  {
    vector<Assoc<Desc,uint32_t> > pairing;
    if(mRes.tPrep!=0.0)
      mRes.tPrep+=mpClassifier->prepare();
    else
      mRes.tPrep=mpClassifier->prepare();
    mRes.tPair=dynamic_cast<Classifier<BriefData,uint32_t>* >(mpClassifier)->pair(queryDs, pairing);
//    cout<<"Paired "<<queryDs.size()<<" queries"<<endl;
    mRes.querySize=queryDs.size();
    mRes.mapSize=mpClassifier->getMapDs().size();
    return AttemptRecovery(pairing, lastPose, maxDist);
  }

  bool AttemptRecovery(vector<Assoc<Desc,uint32_t> >& pairing, SE3<double>& lastPose, double maxDist=5.0)
  {
    //    for (uint32_t i=0; i<mPairing.size(); ++i)
    //      cout<<mPairing[i].d<<" "<<mPairing[i].q<<" <-> "<<mPairing[i].m<<endl;

#ifdef OUTPUT_RELOCALIZATION_FILES
    cout<<"Output "<<mMap.vpKeyFrames.size()<<" keyframes and corresponding points"<<endl;
    vector<bool> drawn(mMap.vpKeyFrames.size(),false);
    vector<ofstream*> pointOut;
    for(size_t j=0; j<mMap.vpKeyFrames.size(); ++j)
    { // open necessary ofstreams to write point correspondences
      sprintf(buf,"../../data/kFpoints%03d.txt",int(j));
      pointOut.push_back(new ofstream(buf));
      cout<< "opening file "<<buf<<endl;
    }
    for(size_t i=0; i<pairing.size();++i)
      if(pairing[i].mp1!=NULL)
      { // If we found a pairing
        for(size_t j=0; j<mMap.vpKeyFrames.size(); ++j)
          if(pairing[i].mp2->pBData->mpKF==mMap.vpKeyFrames[j])
          {
            if(!drawn[j]){
              sprintf(buf,"../../data/kFrame%03d.jpg",int(j));
              CVD::img_save(pairing[i].mp2->pBData->mpKF->aLevels[0].im,buf);
              cout<< "saving image "<<buf<<endl;
              { // output the features directly associated with the KeyFrame
                sprintf(buf,"../../data/kFpointsDirect%03d.txt",int(j));
                ofstream o(buf);
                std::map<MapPoint*, Measurement> meas= pairing[i].mp2->pBData->mpKF->mMeasurements;
                std::map<MapPoint*, Measurement>::iterator it;
                for (it = meas.begin(); it != meas.end(); it++)
                {
                  o << it->second.v2RootPos[0] << "\t"<< it->second.v2RootPos[1] <<
                      "\t" << it->second.nLevel << endl;
                  o.flush();
                }
                o.flush(); o.close();
              }
              drawn[j]=true;
            }
            { // output the feature positions associated directly with the pairings.
              // first coordinates are form the current frame second coordinates are from the KeyFrame
              (*pointOut[j])<<pairing[i].mp1->pBData->v2p[0]<<" "<<pairing[i].mp1->pBData->v2p[1]<<
                  " "<<pairing[i].mp2->pBData->v2p[0]<<" "<<pairing[i].mp2->pBData->v2p[1]<<" "<<pairing[i].dist<<endl;
              //            (*pointOut[j])<<;nnPairing[i].mp1->pTData->v2Found[0]<<" "<<nnPairing[i].mp1->pTData->v2Found[1]<<
              //                " "<<nnPairing[i].mp2->pTData->v2Found[0]<<" "<<nnPairing[i].mp2->pTData->v2Found[1]<<endl;
            }
          }
      }
    for(size_t j=0; j<mMap.vpKeyFrames.size(); ++j)
    { // close ofstreams
      pointOut[j]->close();
      delete pointOut[j];
    }

    CVD::img_save(kCurrent.aLevels[0].im,"../../data/currFrame.jpg"); // save current Frame as well
#endif

    // RANSAC
    mse3Best=lastPose;
//    cout<<"Pose before: "<<endl<<mse3Best<<endl;
//    cout<<"Ransac find model."<<endl;
    // Ransac estimates the inverse transformation (wTc) of mse3Best which is cTw

    Timer t0;
    mse3Best=mRansac.find(pairing).inverse();
    mRes.tRANSAC=t0.toc();

    t0.tic();
    // The next two data structs contain the list of points which will next
    // be searched for in the image, and then used in pose update.
    vector<Assoc<BriefData,uint32_t> > vIterationSet;
    Mod weakerModel(mpModel->getCamModel(), Mod::CONSENSUS_THR*3.0);
    weakerModel.consensusSet(mse3Best, pairing, vIterationSet);
//    cout<<"Pose after RANSAC: "<<endl<<mse3Best
//        <<" #weakerInliers for MEstimator: "<<vIterationSet.size()<<endl;

//    // And attach to the optimization-set.
//    for(size_t i=0; i<pairing.size();++i)//std::min(size_t(300),nnPairing.size());++i)
//      if(pairing[i].q!=NULL)// && mPairing[i].dist<11)
//        vIterationSet.push_back(&pairing[i]);

    // Which M-estimator are we using?
    int nEstimator = 0;
    static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
    {
      if (strcmp(gvsEstimator->c_str(),"Tukey"))
        nEstimator = 0;
      else if (strcmp(gvsEstimator->c_str() ,"Cauchy"))
        nEstimator = 1;
      else if (strcmp(gvsEstimator->c_str(),"Huber"))
        nEstimator = 2;
      else
      {
        cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber"
            << endl;
        nEstimator = 0;
        *gvsEstimator = "Tukey";
      };
    }
    // Ten gauss-newton pose update iterations.
    Vector<6> v6LastUpdate;
    v6LastUpdate = Zeros;
    double avgWeightedE=FLT_MAX, prevAvgWeightedE=0.5*FLT_MAX;
    const double eps=1e-6;

    // TODO: assumtion here is that the search level is always equal!!!!
    uint32_t nSearchLevel=0;
    double dSqrtInvNoise=1/(1<<nSearchLevel);

    vector<bool> inImage(vIterationSet.size(),true);
    vector<Vector<2> > v2Image(vIterationSet.size());
    vector<Matrix<2,6> > m26Jacobian(vIterationSet.size());

    for (int iter = 0; iter < 10; iter++)
    {
      bool bNonLinearIteration=true; // For a bit of time-saving: don't do full nonlinear
      // reprojection at every iteration - it really isn't necessary!
      //    if (iter == 0 || iter == 4 || iter == 9)
      //      bNonLinearIteration = true; // Even this is probably overkill, the reason we do many
      //    else
      //      // iterations is for M-Estimator convergence rather than
      //      bNonLinearIteration = false; // linearisation effects.

      if (bNonLinearIteration)
      {
        for (unsigned int i = 0; i < vIterationSet.size(); i++)
        {
          //            vIterationSet[i]->ProjectAndDerivs(mse3Best, mCamera); //mse3CamFromWorld pose of the camera
          Vector<3> v3Cam = mse3Best * vIterationSet[i].m->pt.v3WorldPos;
          if(v3Cam[2] < 0.001)
            {inImage[i] = false; continue;}
          Vector<2> v2ImPlane = project(v3Cam);
          if(v2ImPlane*v2ImPlane > mpModel->getCamModel().LargestRadiusInImage() * mpModel->getCamModel().LargestRadiusInImage())
            {inImage[i] = false; continue;}
          v2Image[i] = mpModel->getCamModel().Project(v2ImPlane);
          if(mpModel->getCamModel().Invalid())
            {inImage[i] = false; continue;}
          if(v2Image[i][0] < 0 || v2Image[i][1] < 0 || v2Image[i][0] > imgWidth || v2Image[i][1] > imgHeight)
            {inImage[i] = false; continue;}
          inImage[i] = true;
          Matrix<2> m2CamDerivs = mpModel->getCamModel().GetProjectionDerivs();

          //            vIterationSet[i]->CalcJacobian();
          double dOneOverCameraZ = 1.0 / v3Cam[2];
          for(int m=0; m<6; m++)
          {
            const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Cam));
            Vector<2> v2CamFrameMotion;
            v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
            v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
            m26Jacobian[i].T()[m] = m2CamDerivs * v2CamFrameMotion;

//            cerr<<v3Cam<<" -> "<<v2ImPlane<<" -> " <<v2Image[i]<<endl;
//            cerr<<"m26Jacobian: "<<m26Jacobian[i]<<endl;
//            cerr<<"v2CamFrameMotion: "<<v2CamFrameMotion<<endl;
//            cerr<<"m2CamDerivs: "<<m2CamDerivs <<endl;
            assert(!isnan(m26Jacobian[i][0][0]));
          };
        }
      }
      else{
        for (unsigned int i = 0; i < vIterationSet.size(); i++)
        {
          //            vIterationSet[i]->LinearUpdate(v6LastUpdate);
          v2Image[i] += m26Jacobian[i] * v6LastUpdate;
        }
      };

      // Again, an M-Estimator hack beyond the fifth iteration.
      double dOverrideSigma = 0.0;
      //    if (iter > 5) dOverrideSigma = 16.0;

      // Calculate and update pose; also store update vector for linear iteration updates.
//      Vector<6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma, iter>2, &avgWeightedE);

      // Find the covariance-scaled rEeprojection error for each measurement.
      // Also, store the square of these quantities for M-Estimator sigma squared estimation.
      //cout<<"Errors Covscaled: "<<endl;
      vector<double> vdErrorSquared; vdErrorSquared.reserve(vIterationSet.size());
      vector<Vector<2> > v2ErrorCovScaled; v2ErrorCovScaled.reserve(vIterationSet.size());
      vector<uint32_t> indexConv(vIterationSet.size());
      for (unsigned int i = 0,j=0; i < vIterationSet.size(); i++)
      {
        indexConv[i]=0;
        if(!inImage[i]) continue;
        v2ErrorCovScaled.push_back(dSqrtInvNoise * (vIterationSet[i].q->v2p - v2Image[i])); // found position - projected position
        //cout<<TD.v2Found<<"-"<<TD.v2Image<<"->"<<TD.v2Error_CovScaled<<"; ";
        uint32_t end=v2ErrorCovScaled.size()-1;
        vdErrorSquared.push_back(v2ErrorCovScaled[end] * v2ErrorCovScaled[end]);
//        cout<<"vdErrorSquared: "<<vdErrorSquared[vdErrorSquared.size()-1]<<endl;
        assert(!isnan(vdErrorSquared[vdErrorSquared.size()-1]));

        indexConv[i]=j;
        ++j;
      };

      Vector<6> v6Update;
      if (vdErrorSquared.size() > 0)
      {
        // What is the distribution of errors?
        double dSigmaSquared;
        if (dOverrideSigma > 0)
          dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
        else
        {
          if (nEstimator == 0)
            dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
          else if (nEstimator == 1)
            dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
          else
            dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
        }

        // The TooN WLSCholesky class handles reweighted least squares.
        // It just needs errors and jacobians.
        WLS<6> wls;
        wls.add_prior(100.0); // Stabilising prior
        size_t nPointsInEstimator=0;
        double E=0.0, weightedE=0.0, N=0.0;
        for (unsigned int i = 0; i < vIterationSet.size(); i++)
        {
          if(!inImage[i]) continue;
          double dErrorSq = vdErrorSquared[indexConv[i]];
          double dWeight;

          if (nEstimator == 0)
            dWeight = Tukey::Weight(dErrorSq, dSigmaSquared);
          else if (nEstimator == 1)
            dWeight = Cauchy::Weight(dErrorSq, dSigmaSquared);
          else
            dWeight = Huber::Weight(dErrorSq, dSigmaSquared);

          nPointsInEstimator++;
          E+=sqrt(dErrorSq); N++;
          weightedE+=sqrt(dErrorSq)*dWeight;

          Matrix<2, 6> &m26Jac = m26Jacobian[i];
//          cerr<<"indexConv: "<<m26Jacobian[i]<<endl;
//          assert(!isnan(m26Jacobian[i][0][0]));
          wls.add_mJ(v2ErrorCovScaled[indexConv[i]][0], dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
          wls.add_mJ(v2ErrorCovScaled[indexConv[i]][1], dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
        }
//        cout<<endl<<" # of points in M-Estimator("<<*gvsEstimator<<"): "
//            <<nPointsInEstimator
//            <<" out of "<<vIterationSet.size()<<endl
//            <<" sum of errors: "<<E<<" mean error="<<E/N<<endl
//            <<" sum of weighted errors="<<weightedE
//            <<" mean of weighted errors="<<weightedE/N<<endl;
        avgWeightedE=weightedE/N;
        wls.compute();
        v6Update=wls.get_mu();
      }else{ // no valid measurements
        cerr<<"No v2ErrorCovScaleds!"<<endl;
        v6Update=makeVector(0, 0, 0, 0, 0, 0);
      }
      mse3Best = SE3<>::exp(v6Update) * mse3Best;
//      cout<<"Update pose with "<<endl<<SE3<>::exp(v6Update)
//              <<" weightedError change="<<prevAvgWeightedE-avgWeightedE
//              <<" @iteration "<<iter<<endl;
      v6LastUpdate = v6Update;

      if(fabs(prevAvgWeightedE-avgWeightedE)<eps) break;
      prevAvgWeightedE=avgWeightedE;
    };

//    cout<<"Pose after MEstimator: "<<endl<<mse3Best<<endl;
//    cout.flush();
    mInlierCount=mpModel->consensusSize(mse3Best,pairing);
//    cout<<"Consensus size now: "<<mInlierCount<<endl;

    Vector<3> dp=lastPose.get_translation()-mse3Best.get_translation();

    mRes.tMEst=t0.toc();
    mRes.inliers=mInlierCount;
    mRes.wTc=mse3Best.inverse();

    double dist=sqrt(dp[0]*dp[0]+dp[1]*dp[1]+dp[2]*dp[2]);
    // TODO: use motion model here!!
    if(isnan(mse3Best.get_translation()[0]) || dist>maxDist)
      return false;
    else
      return true;
  };

  uint32_t getInlierCount() const {return mInlierCount;};
  const Prosac<FourPoint<ATANCamera,Desc>,Desc>& getRansac() const {return mRansac;};
  //  const vector<Assoc<Desc,uint32_t> >& getPairing() const {return mPairing;};

protected:


  LSH<BriefData,uint32_t>* mpClassifier; // classifier to use for search
  Random mRnd;
  Mod* mpModel;
  //TODO: Prosac or RANSAC?
  RANSAC mRansac;
  //  vector<Assoc<Desc,uint32_t> > mPairing;
  uint32_t mInlierCount;

  const double imgWidth;
  const double imgHeight;

  // Project point into image given certain pose and camera.
  // This can bail out at several stages if the point
  // will not be properly in the image.
//  inline void Project(const SE3<> &se3CFromW, ATANCamera &Cam)
//  {
//
//  }
};

class PtamBriefRansacRelocaliser : public BriefRansacRelocaliser<Prosac<FourPoint<ATANCamera,BriefData>,BriefData>,FourPoint<ATANCamera,BriefData>,BriefData>
{
public:

  PtamBriefRansacRelocaliser(const Map& map, FourPoint<ATANCamera,BriefData>* model,
      const LshParams& lshParams,
      const RansacParams& ransacParams,
      Feature_t featureType)
  : BriefRansacRelocaliser<Prosac<FourPoint<ATANCamera,BriefData>,BriefData>,FourPoint<ATANCamera,BriefData>,BriefData> (
      model,
      new LSH<BriefData,uint32_t>(mapPoint2BriefData(map.vpPoints, mMapBDs),lshParams),
      ransacParams),
      mMap(map),
      mFeatureType(featureType)
  {};

  ~PtamBriefRansacRelocaliser()
  {};

  bool AttemptRecovery(KeyFrame &kCurrent)
  {
    Timer t0;
//    cout<<"BriefRansacRelocaliser::AttemptRecovery"<<endl;
    double maxDist=5.0;
    // TODO: does this work? If we lost track we should need to extract features explicitly?
    // KeyFrame will not have any measurements stored since it wants them associated with
    // MapPoints
    vector<MapPoint*> queryMPs;

    BinaryExtractor be(mFeatureType);
    be.extractForFastCorners(kCurrent,queryMPs); // needs object to automatically delete extracted points at the end of this method
    mRes.tExtract=t0.toc();

    t0.tic();
    // find the set of mappoints which were observed in keyframes that are in
    // a circle of RELEVANCE_RADIUS m around the current pose.
    mMapBDs.clear(); mMapBDs.reserve(mMap.vpPoints.size()*2);
    Vector<3,double> curPos=kCurrent.se3CfromW.inverse().get_translation();
    map<KeyFrame*,bool> relevantKFs;
    for(uint32_t i=0; i<mMap.vpKeyFrames.size(); ++i)
    {
      Vector<3,double> pos=mMap.vpKeyFrames[i]->se3CfromW.inverse().get_translation();
      if(norm(pos-curPos)<RELEVANCE_RADIUS)
        relevantKFs[mMap.vpKeyFrames[i]]=true;
    }

    uint32_t nBriefDescs=0;
    for(uint32_t i=0; i<mMap.vpPoints.size(); ++i)
    {
      nBriefDescs+=mMap.vpPoints[i]->bData.size();
      for(uint32_t j=0; j<mMap.vpPoints[i]->bData.size(); ++j)
        if(relevantKFs.find(mMap.vpPoints[i]->bData[j]->mpKF) != relevantKFs.end())
          mMapBDs.push_back(mMap.vpPoints[i]->bData[j]);
    }

    // exchange the classifier with a version that has updated mapPoints!
    LshParams lshParams=mpClassifier->getParams();
    delete mpClassifier;
    mpClassifier= new LSH<BriefData,uint32_t>(mMapBDs,lshParams);
    mRes.tPrep=t0.toc();

#ifndef TIME_MEASUREMENT
    cout<<" Using "<<100.0*double(mMapBDs.size())/double(nBriefDescs)
        << "% of BRIEF descriptors in the map"<<endl;
    cout<<"tExtract="<<mRes.tExtract<<endl;
#endif

    bool success=AttemptRecovery(queryMPs, kCurrent.se3CfromW, maxDist);
    return success;
  };
  //
  bool AttemptRecovery(vector<MapPoint*>& queryMPs, SE3<double>& lastPose, double maxDist=5.0)
  {
    vector<BriefData*> queryBDs; mapPoint2BriefData(queryMPs, queryBDs);
    return BriefRansacRelocaliser<Prosac<FourPoint<ATANCamera,BriefData>,BriefData>,
        FourPoint<ATANCamera,BriefData>,BriefData>::AttemptRecovery(queryBDs,lastPose,maxDist);
  }

private:
  const Map& mMap;
  vector<BriefData*> mMapBDs;

  Feature_t mFeatureType;

  static const double RELEVANCE_RADIUS=5.0; // [m] within this radius keyframes are taken into consideration for building the list of points to match against
};


template<class Mod, class Desc>
class BriefMEstRelocaliser : public Relocaliser
{
public:

  BriefMEstRelocaliser(Mod* model,
      LSH<BriefData,uint32_t>* classifier,
      double width=640, double height=480)
  : Relocaliser(model->getCamModel()),
    mpClassifier(classifier),
    mpModel(model),
    imgWidth(width), imgHeight(height)
  {};

  ~BriefMEstRelocaliser()
  {
    delete mpModel;
    delete mpClassifier;
  };

  bool AttemptRecovery(vector<Desc*>& queryDs, SE3<double>& lastPose, double maxDist=5.0)
  {
    vector<Assoc<Desc,uint32_t> > pairing;
    mRes.tPrep=mpClassifier->prepare();
    mRes.tPair=dynamic_cast<Classifier<BriefData,uint32_t>* >(mpClassifier)->pair(queryDs, pairing);
    cout<<"Paired "<<queryDs.size()<<" queries"<<endl;
    mRes.querySize=queryDs.size();
    mRes.mapSize=mpClassifier->getMapDs().size();
    return AttemptRecovery(pairing, lastPose, maxDist);
  }

  bool AttemptRecovery(vector<Assoc<Desc,uint32_t> >& pairing, SE3<double>& lastPose, double maxDist=5.0)
  {
    //    for (uint32_t i=0; i<mPairing.size(); ++i)
    //      cout<<mPairing[i].d<<" "<<mPairing[i].q<<" <-> "<<mPairing[i].m<<endl;

#ifdef OUTPUT_RELOCALIZATION_FILES
    cout<<"Output "<<mMap.vpKeyFrames.size()<<" keyframes and corresponding points"<<endl;
    vector<bool> drawn(mMap.vpKeyFrames.size(),false);
    vector<ofstream*> pointOut;
    for(size_t j=0; j<mMap.vpKeyFrames.size(); ++j)
    { // open necessary ofstreams to write point correspondences
      sprintf(buf,"../../data/kFpoints%03d.txt",int(j));
      pointOut.push_back(new ofstream(buf));
      cout<< "opening file "<<buf<<endl;
    }
    for(size_t i=0; i<pairing.size();++i)
      if(pairing[i].mp1!=NULL)
      { // If we found a pairing
        for(size_t j=0; j<mMap.vpKeyFrames.size(); ++j)
          if(pairing[i].mp2->pBData->mpKF==mMap.vpKeyFrames[j])
          {
            if(!drawn[j]){
              sprintf(buf,"../../data/kFrame%03d.jpg",int(j));
              CVD::img_save(pairing[i].mp2->pBData->mpKF->aLevels[0].im,buf);
              cout<< "saving image "<<buf<<endl;
              { // output the features directly associated with the KeyFrame
                sprintf(buf,"../../data/kFpointsDirect%03d.txt",int(j));
                ofstream o(buf);
                std::map<MapPoint*, Measurement> meas= pairing[i].mp2->pBData->mpKF->mMeasurements;
                std::map<MapPoint*, Measurement>::iterator it;
                for (it = meas.begin(); it != meas.end(); it++)
                {
                  o << it->second.v2RootPos[0] << "\t"<< it->second.v2RootPos[1] <<
                      "\t" << it->second.nLevel << endl;
                  o.flush();
                }
                o.flush(); o.close();
              }
              drawn[j]=true;
            }
            { // output the feature positions associated directly with the pairings.
              // first coordinates are form the current frame second coordinates are from the KeyFrame
              (*pointOut[j])<<pairing[i].mp1->pBData->v2p[0]<<" "<<pairing[i].mp1->pBData->v2p[1]<<
                  " "<<pairing[i].mp2->pBData->v2p[0]<<" "<<pairing[i].mp2->pBData->v2p[1]<<" "<<pairing[i].dist<<endl;
              //            (*pointOut[j])<<;nnPairing[i].mp1->pTData->v2Found[0]<<" "<<nnPairing[i].mp1->pTData->v2Found[1]<<
              //                " "<<nnPairing[i].mp2->pTData->v2Found[0]<<" "<<nnPairing[i].mp2->pTData->v2Found[1]<<endl;
            }
          }
      }
    for(size_t j=0; j<mMap.vpKeyFrames.size(); ++j)
    { // close ofstreams
      pointOut[j]->close();
      delete pointOut[j];
    }

    CVD::img_save(kCurrent.aLevels[0].im,"../../data/currFrame.jpg"); // save current Frame as well
#endif

    mse3Best=lastPose;
    cout<<"Pose before: "<<endl<<mse3Best<<endl;
    // The next two data structs contain the list of points which will next
    // be searched for in the image, and then used in pose update.
    vector<Assoc<BriefData,uint32_t> > vIterationSet=pairing;
        cout<<" # MEstimator pairings: "<<vIterationSet.size()<<endl;

//    // And attach to the optimization-set.
//    for(size_t i=0; i<pairing.size();++i)//std::min(size_t(300),nnPairing.size());++i)
//      if(pairing[i].q!=NULL)// && mPairing[i].dist<11)
//        vIterationSet.push_back(&pairing[i]);

    // Which M-estimator are we using?
    int nEstimator = 0;
    static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
    {
      if (strcmp(gvsEstimator->c_str(),"Tukey"))
        nEstimator = 0;
      else if (strcmp(gvsEstimator->c_str() ,"Cauchy"))
        nEstimator = 1;
      else if (strcmp(gvsEstimator->c_str(),"Huber"))
        nEstimator = 2;
      else
      {
        cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber"
            << endl;
        nEstimator = 0;
        *gvsEstimator = "Tukey";
      };
    }
    // Ten gauss-newton pose update iterations.
    Vector<6> v6LastUpdate;
    v6LastUpdate = Zeros;
    double avgWeightedE=FLT_MAX, prevAvgWeightedE=0.5*FLT_MAX;
    const double eps=1e-6;

    // TODO: assumtion here is that the search level is always equal!!!!
    uint32_t nSearchLevel=0;
    double dSqrtInvNoise=1/(1<<nSearchLevel);

    vector<bool> inImage(vIterationSet.size(),true);
    vector<Vector<2> > v2Image(vIterationSet.size());
    vector<Matrix<2,6> > m26Jacobian(vIterationSet.size());

    for (int iter = 0; iter < 30; iter++)
    {
      bool bNonLinearIteration=true; // For a bit of time-saving: don't do full nonlinear
      // reprojection at every iteration - it really isn't necessary!
      //    if (iter == 0 || iter == 4 || iter == 9)
      //      bNonLinearIteration = true; // Even this is probably overkill, the reason we do many
      //    else
      //      // iterations is for M-Estimator convergence rather than
      //      bNonLinearIteration = false; // linearisation effects.

      if (bNonLinearIteration)
      {
        for (unsigned int i = 0; i < vIterationSet.size(); i++)
        {
          //            vIterationSet[i]->ProjectAndDerivs(mse3Best, mCamera); //mse3CamFromWorld pose of the camera
          Vector<3> v3Cam = mse3Best * vIterationSet[i].m->pt.v3WorldPos;
          cout<<v3Cam<<endl;
          if(v3Cam[2] < 0.001)
            {inImage[i] = false; continue;}
          Vector<2> v2ImPlane = project(v3Cam);
          cout<<v2ImPlane<<" "<<v2ImPlane*v2ImPlane<< " !< "<<mpModel->getCamModel().LargestRadiusInImage() * mpModel->getCamModel().LargestRadiusInImage()<<endl;
          if(v2ImPlane*v2ImPlane > mpModel->getCamModel().LargestRadiusInImage() * mpModel->getCamModel().LargestRadiusInImage())
            {inImage[i] = false; continue;}
          v2Image[i] = mpModel->getCamModel().Project(v2ImPlane);
          if(mpModel->getCamModel().Invalid())
            {inImage[i] = false; continue;}
          cout<<v2Image[i]<<endl;
          if(v2Image[i][0] < 0 || v2Image[i][1] < 0 || v2Image[i][0] > imgWidth || v2Image[i][1] > imgHeight)
            {inImage[i] = false; continue;}
          inImage[i] = true;
          Matrix<2> m2CamDerivs = mpModel->getCamModel().GetProjectionDerivs();

          //            vIterationSet[i]->CalcJacobian();
          double dOneOverCameraZ = 1.0 / v3Cam[2];
          for(int m=0; m<6; m++)
          {
            const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Cam));
            Vector<2> v2CamFrameMotion;
            v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
            v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
            m26Jacobian[i].T()[m] = m2CamDerivs * v2CamFrameMotion;

//            cerr<<v3Cam<<" -> "<<v2ImPlane<<" -> " <<v2Image[i]<<endl;
//            cerr<<"m26Jacobian: "<<m26Jacobian[i]<<endl;
//            cerr<<"v2CamFrameMotion: "<<v2CamFrameMotion<<endl;
//            cerr<<"m2CamDerivs: "<<m2CamDerivs <<endl;
            assert(!isnan(m26Jacobian[i][0][0]));
          };
        }
      }
      else{
        for (unsigned int i = 0; i < vIterationSet.size(); i++)
        {
          //            vIterationSet[i]->LinearUpdate(v6LastUpdate);
          v2Image[i] += m26Jacobian[i] * v6LastUpdate;
        }
      };

      // Again, an M-Estimator hack beyond the fifth iteration.
      double dOverrideSigma = 0.0;
      //    if (iter > 5) dOverrideSigma = 16.0;

      // Calculate and update pose; also store update vector for linear iteration updates.
//      Vector<6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma, iter>2, &avgWeightedE);

      // Find the covariance-scaled rEeprojection error for each measurement.
      // Also, store the square of these quantities for M-Estimator sigma squared estimation.
      //cout<<"Errors Covscaled: "<<endl;
      vector<double> vdErrorSquared; vdErrorSquared.reserve(vIterationSet.size());
      vector<Vector<2> > v2ErrorCovScaled; v2ErrorCovScaled.reserve(vIterationSet.size());
      vector<uint32_t> indexConv(vIterationSet.size());
      for (unsigned int i = 0,j=0; i < vIterationSet.size(); i++)
      {
        indexConv[i]=0;
        if(!inImage[i]) continue;
        v2ErrorCovScaled.push_back(dSqrtInvNoise * (vIterationSet[i].q->v2p - v2Image[i])); // found position - projected position
        //cout<<TD.v2Found<<"-"<<TD.v2Image<<"->"<<TD.v2Error_CovScaled<<"; ";
        uint32_t end=v2ErrorCovScaled.size()-1;
        vdErrorSquared.push_back(v2ErrorCovScaled[end] * v2ErrorCovScaled[end]);
//        cout<<"vdErrorSquared: "<<vdErrorSquared[vdErrorSquared.size()-1]<<endl;
        assert(!isnan(vdErrorSquared[vdErrorSquared.size()-1]));

        indexConv[i]=j;
        ++j;
      };

      Vector<6> v6Update;
      if (vdErrorSquared.size() > 0)
      {
        // What is the distribution of errors?
        double dSigmaSquared;
        if (dOverrideSigma > 0)
          dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
        else
        {
          if (nEstimator == 0)
            dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
          else if (nEstimator == 1)
            dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
          else
            dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
        }

        // The TooN WLSCholesky class handles reweighted least squares.
        // It just needs errors and jacobians.
        WLS<6> wls;
        wls.add_prior(100.0); // Stabilising prior
        size_t nPointsInEstimator=0;
        double E=0.0, weightedE=0.0, N=0.0;
        for (unsigned int i = 0; i < vIterationSet.size(); i++)
        {
          if(!inImage[i]) continue;
          double dErrorSq = vdErrorSquared[indexConv[i]];
          double dWeight;

          if (nEstimator == 0)
            dWeight = Tukey::Weight(dErrorSq, dSigmaSquared);
          else if (nEstimator == 1)
            dWeight = Cauchy::Weight(dErrorSq, dSigmaSquared);
          else
            dWeight = Huber::Weight(dErrorSq, dSigmaSquared);

          nPointsInEstimator++;
          E+=sqrt(dErrorSq); N++;
          weightedE+=sqrt(dErrorSq)*dWeight;

          Matrix<2, 6> &m26Jac = m26Jacobian[i];
//          cerr<<"indexConv: "<<m26Jacobian[i]<<endl;
//          assert(!isnan(m26Jacobian[i][0][0]));
          wls.add_mJ(v2ErrorCovScaled[indexConv[i]][0], dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
          wls.add_mJ(v2ErrorCovScaled[indexConv[i]][1], dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
        }
        cout<<endl<<" # of points in M-Estimator("<<*gvsEstimator<<"): "
            <<nPointsInEstimator
            <<" out of "<<vIterationSet.size()<<endl
            <<" sum of errors: "<<E<<" mean error="<<E/N<<endl
            <<" sum of weighted errors="<<weightedE
            <<" mean of weighted errors="<<weightedE/N<<endl;
        avgWeightedE=weightedE/N;
        wls.compute();
        v6Update=wls.get_mu();
      }else{ // no valid measurements
        cerr<<"No v2ErrorCovScaleds!"<<endl;
        v6Update=makeVector(0, 0, 0, 0, 0, 0);
      }
      mse3Best = SE3<>::exp(v6Update) * mse3Best;
      cout<<"Update pose with "<<endl<<SE3<>::exp(v6Update)
              <<" weightedError change="<<prevAvgWeightedE-avgWeightedE
              <<" @iteration "<<iter<<endl;
      v6LastUpdate = v6Update;

      if(fabs(prevAvgWeightedE-avgWeightedE)<eps) break;
      prevAvgWeightedE=avgWeightedE;
    };

    cout<<"Pose after MEstimator: "<<endl<<mse3Best<<endl;
    cout.flush();
    mInlierCount=mpModel->consensusSize(mse3Best,pairing);
    cout<<"Consensus size now: "<<mInlierCount<<endl;

    Vector<3> dp=lastPose.get_translation()-mse3Best.get_translation();

    mRes.inliers=mInlierCount;
    mRes.wTc=mse3Best.inverse();


    double dist=sqrt(dp[0]*dp[0]+dp[1]*dp[1]+dp[2]*dp[2]);
    // TODO: use motion model here!!
    if(isnan(mse3Best.get_translation()[0]) || dist>maxDist)
      return false;
    else
      return true;
  };

  uint32_t getInlierCount() const {return mInlierCount;};

protected:

//  vector<BriefData*> mMapBDs;
  LSH<BriefData,uint32_t>* mpClassifier; // classifier to use for search
  Mod* mpModel;
  //  vector<Assoc<Desc,uint32_t> > mPairing;
  uint32_t mInlierCount;

  const double imgWidth;
  const double imgHeight;

};

#endif /* BRIEFRELOCALIZER_HPP_ */
