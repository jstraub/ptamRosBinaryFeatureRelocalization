/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef BRIEFIR_HPP_
#define BRIEFIR_HPP_

#include <results.hpp>
#include <lshKBM.hpp>
#include <imageset.h>
#include <db.h>
#include <lsh.hpp>

#include <TooN/TooN.h>
#include <sstream>
#include <string>
#include <algorithm>

using namespace std;
using namespace TooN;

struct IrScore
{
  IrScore(uint32_t img_id, float score_) : imgId(img_id), score(score_)
  {};
  IrScore(const IrScore& irS) : imgId(irS.imgId), score(irS.score)
  {};

  uint32_t imgId;
  float score;

  bool operator< (const IrScore& b) const{
    return (score<b.score);
  }
};

struct IRConfig
{
  bool constrainToArea; // if true the next two are required as well!
  Vector<3,double> center;
  double radius;
  uint32_t nThreadsMax; // how many threads at max

};

struct IRResult
{

};

ImageSet* loadImageSet(FeatureDatabase& fdb, LshKBM& lshKBM,
    vector<uint32_t>& imageIds, string pathToSet);

class BriefIR
{
public:
  BriefIR(FeatureDatabase& fdb, LshKBM& lshKBM, ImageSet* imgSet, const IRConfig& config)
  : mFdb(fdb), mLshKBM(lshKBM), mpImgSet(imgSet), mConfig(config)
  {
    if(mConfig.constrainToArea)
    {
      { // find frame Ids which are in the approximate area of the queries to later constrain the
        // search.
        Vector<3,double> center; center[0]=-156.0; center[1]=-14.2; center[2]=.0;
        double radius=30.0;
        frameIDsInArea(fdb, mFrameIDsInQueryArea, center, radius);
      }
    }
  };
  virtual ~BriefIR()
  {};

  static void frameIDsInArea(FeatureDatabase& fdb, map<uint32_t,bool>& frameIDs,
      Vector<3,double> center, double radius)
  {
    const db_index_t& dbIndex=fdb.index();// for each image
    // iterate over images in the index
    for(uint32_t i=0; i<dbIndex.size(); ++i){//dbIndex.size(); ++i){
      const meta_file_entry* dbMeta=fdb.firstMetaEntry(i); // for each feature
      // iterate over the features found in image with index i
      Vector<3,double> meanPos; meanPos[0]=0.0; meanPos[1]=0.0; meanPos[2]=0.0;
      for(int j=0; j<dbIndex[i].desc_count; ++j)
      {
        Vector<3,double> pos;
        pos[0]=dbMeta[j].world_x; pos[1]=dbMeta[j].world_y; pos[2]=dbMeta[j].world_z;
        meanPos+=pos;
      }
      meanPos/=dbIndex[i].desc_count;
      if(norm(meanPos-center)<radius)
        frameIDs[i]=true;
      else
        frameIDs[i]=false;
    }
  };

  template<class Desc>
  void getImgScores(const vector<Desc*>& queryMPs,
      vector<IrScore>& scores, uint32_t kBest) const
  {
    vector<uint32_t> codewords;
    for(size_t i=0; i<queryMPs.size(); ++i){
      LshKBMCallback qc;
      QuantizerContext qtxt;
      mLshKBM.quantizeForQuery(qc,queryMPs[i],qtxt);
      codewords.push_back(qc.mVisualWord);
    };
    cout<<"Searching for "<<codewords.size()<< " codewords"<<endl;
    //    for(size_t i=0; i<codewords.size(); ++i){
    //      cout<<codewords[i]<<" ";
    //    }; cout<<endl;
    QueryContext qtxt;
    CvMat* pScoresMat=mpImgSet->search(codewords,qtxt);
    assert(pScoresMat->cols>=0);
    cout<<pScoresMat->rows<<"x"<<pScoresMat->cols<<endl;
    for(uint32_t i=0; i<uint32_t(pScoresMat->cols); ++i)
    {
      if(!mConfig.constrainToArea ||
          (mConfig.constrainToArea &&
              ((mFrameIDsInQueryArea.find(i)!=mFrameIDsInQueryArea.end()) &&
                  mFrameIDsInQueryArea.find(i)->second )))
        scores.push_back(IrScore(i,cvmGet(pScoresMat,0,i)));
    }
    cvReleaseMat(&pScoresMat);
    //    cout<<"Scores:"<<endl;
    //    for(uint32_t i=0; i<scores.size(); ++i)
    //      cout<<scores[i].score<<" @"<<scores[i].imgId<<"\t";
    //    cout<<endl;

    uint32_t nFeats=0;
    // sort to find 10 best ones easily - can use partial sort here
//    partial_sort(scores.begin(),scores.begin()+kBest,scores.end());
    sort(scores.begin(),scores.end());
    scores.resize(kBest,IrScore(0,0));

    cout<<kBest<<" Best scores:"<<endl;
    for(uint32_t i=0; i<kBest; ++i)
    {
      cout<<scores[i].score<<" @"<<scores[i].imgId<<"\t"
          <<mFdb.index(scores[i].imgId).fn
          <<"\t#features="<<mFdb.index(scores[i].imgId).desc_count<<endl;
      nFeats+=mFdb.index(scores[i].imgId).desc_count;
    }
    cout<<" ---- total #features=" <<nFeats<<endl;

  };

protected:
  FeatureDatabase& mFdb;
  LshKBM& mLshKBM;
  ImageSet* mpImgSet;
  IRConfig mConfig;
  map<uint32_t,bool> mFrameIDsInQueryArea;
private:
};

class BriefVirtViewsLoc : public BriefIR
{
public:
  BriefVirtViewsLoc(FeatureDatabase& fdb, LshKBM& lshKBM, ImageSet* imgSet, const IRConfig& config)
  : BriefIR(fdb, lshKBM, imgSet, config)
  {
    parseVVPoses(mVVPoses);
  };

  ~BriefVirtViewsLoc()
  {};

  vector<LocResult> search(const vector<BriefDesc*>& queryMPs,
      uint32_t kBest=10) const
  {
    vector<LocResult> locRes;
    vector<IrScore> scores;
    getImgScores(queryMPs,scores,kBest);

    const db_index_t& dbIndex=this->mFdb.index();
    for(uint32_t i=0; i<kBest; ++i)
    {
      Vector<3,double> pos;
      pos[0]=mVVPoses[scores[i].imgId][0];
      pos[1]=mVVPoses[scores[i].imgId][1];
      pos[2]=0.0;
      Vector<3,double> so3;
      so3[0]=0.0; so3[1]=0.0;
      so3[2]=mVVPoses[scores[i].imgId][2];
      SE3<double> wTc(SO3<double>(so3),pos);
      cout<<wTc;
      LocResult res; res.wTc=wTc;

      // search for surrounding images
      vector<uint32_t> ids;
      findSurroundingImgs(scores[i].imgId, ids, 1.0, 15.0);
      for(uint32_t id=0; id<ids.size(); ++id)
      {
        cout<<"  "<<ids[id]<<": "<<mVVPoses[ids[id]]
            <<" #desc="<<dbIndex[ids[id]].desc_count<<endl;
//        uint8_t* desc = ((uint8_t*)this->mFdb.firstDescriptor(ids[id])); // for each feature
        const meta_file_entry* dbMeta=this->mFdb.firstMetaEntry(ids[id]); // for each feature
        for(int j=0; j<dbIndex[ids[id]].desc_count; ++j)
        {// iterate over the features found in image with index i
          Vector<3,double> pos;
          pos[0]=dbMeta[j].world_x; pos[1]=dbMeta[j].world_y; pos[2]=dbMeta[j].world_z;
//          MapPoint* mp= new MapPoint();
//          BriefDataM* bD = new BriefDataM(mp,&desc[j*dims]); // dont copy data just set pointer to the data
//          bD->v2p[0]=dbMeta[j].x; bD->v2p[1]=dbMeta[j].y;
//          mp->v3WorldPos=pos;
//          briefs.push_back(bD);
//          briefFrames[bD]=i;
          res.p3d.push_back(pos);
        }
      }
      locRes.push_back(res);
    }
    return locRes;
  };

  /* find image ids of images that are close by a given id
   *
   */
  void findSurroundingImgs(uint32_t imgID, vector<uint32_t>& ids,
      double r=1.0, double dTheta=15.0) const
  {
    Vector<2,double> pos=mVVPoses[imgID].slice(0,2);
    double theta=mVVPoses[imgID][2];
    for(uint32_t i=0; i<mVVPoses.size(); ++i)
      if(absDAngle(theta,mVVPoses[i][2])<=dTheta &&
          norm(mVVPoses[i].slice(0,2)-pos)<=r)
      {

        ids.push_back(i);
      }
  };

protected:

  vector<Vector<3,double> > mVVPoses; // poses of all virtual view images

  void parseVVPoses(vector<Vector<3,double> >& vvPoses) const
  {
    const db_index_t& imgInd=this->mFdb.index();
    vvPoses.reserve(imgInd.size());

    for(uint32_t i=0; i<imgInd.size(); ++i)
    {
      stringstream ss(imgInd[i].fn);
      Vector<3,double> pose;
      ss>>pose[0]>>pose[1]>>pose[2]; // x y theta(in deg)
      vvPoses.push_back(pose);
    }
  };

  double absDAngle(double a, double b) const
  {
    double c=fabs(a-b);
    return c>180?360-c:c;
  }

  double norm(const Vector<2,double>& a) const
  {
    return sqrt(a[0]*a[0] + a[1]*a[1]);
  }

private:

};



#endif /* BRIEFIR_HPP_ */
