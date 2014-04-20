/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 
#include "BriefExtractor.hpp"

void BinaryExtractor::convertToCvKeyPoints(vector<Candidate>& meas ,vector<cv::KeyPoint>& kps,
    int octave, bool append)
{
  if(!append) kps.clear();
  kps.reserve(meas.size()+kps.size());
  int i=0;
  for(std::vector<Candidate>::iterator it=meas.begin(); it!=meas.end(); ++it,++i)
  {
    cv::KeyPoint kp;
    kp.pt.x=it->v2RootPos[0];
    kp.pt.y=it->v2RootPos[1];
    kp.octave=octave;
    kp.size = 1;
    kp.response = 1;
    kp.class_id = i; // store measurement ID
    kps.push_back(kp);
  }
}

void BinaryExtractor::convertToCvKeyPoints(map<MapPoint*, Measurement>& meas, vector<vector<cv::KeyPoint> >& kps, vector<MapPoint*>& corresp)
{
  kps.clear(); kps.resize(LEVELS,vector<cv::KeyPoint>());
  corresp.clear(); corresp.resize(meas.size(),NULL);
  int i=0;
  for (std::map<MapPoint*, Measurement>::iterator it=meas.begin(); it!=meas.end(); ++it,++i)
  {
    // TrackerData==NULL for the first two frames
    if(it->first->pTData == NULL || it->first->pTData->bFound)
    {
      cv::KeyPoint kp;
      kp.pt.x=it->second.v2RootPos[0];
      kp.pt.y=it->second.v2RootPos[1];
      kp.octave=it->second.nLevel;
      kp.size = 1;
      kp.response = 1;
      kp.class_id = i; // use this variable as the index of the measurement
      kps[kp.octave].push_back(kp); // append to the vector at the correct pyramid level
      corresp[i]=it->first; // corresp[kp.class_id] gives the MapPoint of this measurement
    }
  }
}

void BinaryExtractor::convertToCvKeyPoints(map<MapPoint*, Measurement>& meas, vector<cv::KeyPoint>& kps, vector<MapPoint*>& corresp)
{
  kps.clear(); kps.reserve(meas.size());
  corresp.clear(); corresp.resize(meas.size(),NULL);
  int i=0;
  for (std::map<MapPoint*, Measurement>::iterator it=meas.begin(); it!=meas.end(); ++it,++i)
  {
    // TrackerData==NULL for the first two frames
    if(it->first->pTData == NULL || it->first->pTData->bFound)
    {
      cv::KeyPoint kp;
      kp.pt.x=it->second.v2RootPos[0];
      kp.pt.y=it->second.v2RootPos[1];
      kp.octave=it->second.nLevel;
      kp.size = 1;
      kp.response = 1;
      kp.class_id = i; // use this variable as the index of the measurement
      kps.push_back(kp); // append to the vector
      corresp[i]=it->first; // corresp[kp.class_id] gives the MapPoint of this measurement
    }
  }
}

void BinaryExtractor::extractBRIEF(vector<cv::KeyPoint>& kps, const cv::Mat& img, cv::Mat& bDesc) const
{
  cout<<"Feature Type="<<mFeatureType<<endl;
  if(mFeatureType==BRIEF_T_HIST || mFeatureType==BRIEF_UNSCALED_T_HIST ||
     mFeatureType==BRIEF_T_UPDATE || mFeatureType==BRIEF_UNSCALED_T_UPDATE)
  {
    cv::BriefDescriptorExtractor brief(BriefDesc::BRIEF_K);
    brief.compute(img, kps, bDesc);
    cout<<"BRIEF"<<endl;
  }
  else if(mFeatureType==ORB_T_HIST || mFeatureType==ORB_T_UPDATE)
  {
    cv::OrbDescriptorExtractor orb(500, 2.0f, LEVELS); // This makes ORB extract on the same scales as BRIEF
    orb.compute(img,kps,bDesc);
    cout<<"ORB"<<endl;
  }
  else if(mFeatureType==FREAK_T_HIST || mFeatureType==FREAK_T_UPDATE)
  {
    cv::FREAK freak;
    freak.compute(img,kps,bDesc);
    cout<<"FREAK"<<endl;
  }
  assert(size_t(bDesc.rows)==kps.size());
}

void BinaryExtractor::saveDescriptors(ofstream& outDesc, ofstream& outStat,
    KeyFrame& kF, int32_t frameNr)
{
  cout<< "----------- BriefExtractor::saveDescriptors @frame "<<frameNr<<" -----------"<<endl;
  std::map<MapPoint*, Measurement>& meas=kF.mMeasurements;
  vector<MapPoint*> corresp;
  const double eps=1e-2;
  MapPoint dummyMp; BriefDataS dummyBd(&dummyMp);
  dummyMp.pBData=&dummyBd;

  Timer t0;
  float tDescExtract=0.0f;
  uint32_t nDesc=0;

  if(mFeatureType==BRIEF_T_HIST || mFeatureType==BRIEF_T_UPDATE)
  {
    vector<vector<cv::KeyPoint> > kpsLev;
    convertToCvKeyPoints(meas,kpsLev,corresp);
#ifndef TIME_MEASUREMENT
    cout<<"Copied "<<kpsLev.size()<<" measurements for BRIEF descriptor extraction"<<endl;
    cout<<"BRIEF extraction per level: ";
#endif
    for(size_t l=0; l<LEVELS;++l)
    {
      cv::Mat bDesc;
      t0.tic();
      extractBRIEF(kpsLev[l],kF.aLevels[l].im,bDesc); // extract BRIEF descriptors at specific level l
      tDescExtract+=t0.toc();
      for(uint32_t i=0; i<kpsLev[l].size(); ++i)
      {
        MapPoint* m=corresp[kpsLev[l][i].class_id];
        assert(meas[m].v2RootPos[0]-kpsLev[l][i].pt.x<eps && meas[m].v2RootPos[1]- kpsLev[l][i].pt.y<eps); // they shouldnt lie apart at all!
        meas[m].nLevel; //TODO: Save measuremnts pyramid level as well!!
        dummyBd.v2p=meas[m].v2RootPos;
        dummyBd.mpKF=&kF;
        dummyBd.descriptorFromCvMatRow(&bDesc.data[i*bDesc.cols]);
        dummyMp.v3WorldPos=m->v3WorldPos;
        outDesc<<dummyBd.printForMatlab()<<endl;
      }
#ifndef TIME_MEASUREMENT
      cout<<"Descriptors "<<bDesc.cols<<"x"<<bDesc.rows<<endl;
      cout<<kpsLev[l].size()<<" ";
#endif
      nDesc+=kpsLev[l].size();
    }
  }else if(mFeatureType==BRIEF_UNSCALED_T_HIST || mFeatureType==BRIEF_UNSCALED_T_UPDATE)
  {
    vector<vector<cv::KeyPoint> > kpsLev;
    convertToCvKeyPoints(meas,kpsLev,corresp);
#ifndef TIME_MEASUREMENT
    cout<<"Copied "<<kpsLev.size()<<" measurements for BRIEF descriptor extraction"<<endl;
    cout<<"BRIEF extraction per level: ";
#endif
    for(size_t l=0; l<LEVELS;++l)
    {
      cv::Mat bDesc;
      t0.tic();
      if(l>0)
        for(uint32_t i=0; i<kpsLev[l].size(); ++i)
        { // get the position in full size image.
          kpsLev[l][i].pt.x*=double((1<<l));
          kpsLev[l][i].pt.y*=double((1<<l));
        }
      extractBRIEF(kpsLev[l],kF.aLevels[0].im,bDesc); // extract BRIEF descriptors at specific level l
      tDescExtract+=t0.toc();
      for(uint32_t i=0; i<kpsLev[l].size(); ++i)
      {
        MapPoint* m=corresp[kpsLev[l][i].class_id];
        assert(meas[m].v2RootPos[0]-kpsLev[l][i].pt.x<eps && meas[m].v2RootPos[1]- kpsLev[l][i].pt.y<eps); // they shouldnt lie apart at all!
        meas[m].nLevel; //TODO: Save measuremnts pyramid level as well!!
        dummyBd.v2p=meas[m].v2RootPos;
        dummyBd.mpKF=&kF;
        dummyBd.descriptorFromCvMatRow(&bDesc.data[i*bDesc.cols]);
        dummyMp.v3WorldPos=m->v3WorldPos;
        outDesc<<dummyBd.printForMatlab()<<endl;
      }
#ifndef TIME_MEASUREMENT
      cout<<"Descriptors "<<bDesc.cols<<"x"<<bDesc.rows<<endl;
      cout<<kpsLev[l].size()<<" ";
#endif
      nDesc+=kpsLev[l].size();
    }
  }else if(mFeatureType==ORB_T_HIST || mFeatureType==FREAK_T_HIST ||
           mFeatureType==ORB_T_UPDATE || mFeatureType==FREAK_T_UPDATE)
  {
    cv::Mat bDesc;
    vector<cv::KeyPoint> kps;
    convertToCvKeyPoints(meas,kps,corresp);
    t0.tic();
    extractBRIEF(kps,kF.aLevels[0].im,bDesc); // give it only the 0th pyramid level - pyramids are extracted internally.
    // kps have pyramid level/octave information stored within so ORB/FREAK knows where to extract
    tDescExtract+=t0.toc();
#ifndef TIME_MEASUREMENT
    cout<<"Copied "<<kps.size()<<" measurements for ORB/FREAK descriptor extraction"<<endl;
    cout<<"Descriptors "<<bDesc.cols<<"x"<<bDesc.rows<<endl;
#endif
    for(uint32_t i=0; i<kps.size(); ++i)
    {
      MapPoint* m=corresp[kps[i].class_id];
      assert(meas[m].v2RootPos[0]-kps[i].pt.x<eps && meas[m].v2RootPos[1]- kps[i].pt.y<eps); // they shouldnt lie apart at all!
      meas[m].nLevel; //TODO: Save measurements pyramid level as well!!
      dummyBd.v2p=meas[m].v2RootPos;
      dummyBd.mpKF=&kF;
      dummyBd.descriptorFromCvMatRow(&bDesc.data[i*bDesc.cols]);
      dummyMp.v3WorldPos=m->v3WorldPos;
      outDesc<<dummyBd.printForMatlab()<<endl;
    }
    nDesc=kps.size();
  }

  cout<<endl<< "#Corners="<<meas.size()<<" #Descriptors="<<nDesc<<" #NotExtracted="<<meas.size()-nDesc<<endl;
  outStat << tDescExtract << "\t" << nDesc;

//    else if(mFeatureType==FREAK_T)
//  {
//    cv::Mat bDesc;
//    vector<cv::KeyPoint> kps;
//    convertToCvKeyPoints(meas,kps,corresp);
//    cout<<"Copied "<<kps.size()<<" measurements for FREAK descriptor extraction"<<endl;
//    extractBRIEF(kps,kF.aLevels[0].im,bDesc); // give it only the 0th pyramid level - pyramids are extracted internally.
//    // kps have pyramid level/octave information stored within so FREAK knows where to extract
//
//  }
}

void BinaryExtractor::extractForMapPoints(KeyFrame& kF, int32_t frameNr) const
{
  cout<< "----------- BriefExtractor::extractForMapPoints @frame "<<frameNr<<" -----------"<<endl;
  std::map<MapPoint*, Measurement>& meas=kF.mMeasurements;
  vector<MapPoint*> corresp;
  const double eps=1e-2; // unit is pixels

  if(mFeatureType==BRIEF_T_HIST || mFeatureType==BRIEF_T_UPDATE)
  {
    vector<vector<cv::KeyPoint> > kpsLev;
    convertToCvKeyPoints(meas,kpsLev,corresp);
    cout<<"Copied "<<kpsLev.size()<<" measurements for BRIEF descriptor extraction"<<endl;
    uint32_t nBriefs=0;
    cout<<"BRIEF extraction per level: ";
    Timer t0; // take time
    for(size_t l=0; l<LEVELS;++l)
    {
      cv::Mat bDesc;
      uint32_t nCorners=kpsLev[l].size();
      extractBRIEF(kpsLev[l],kF.aLevels[l].im,bDesc); // extract BRIEF descriptors at specific level l

      for(uint32_t i=0; i<kpsLev[l].size(); ++i)
      {
        MapPoint* m=corresp[kpsLev[l][i].class_id];
        assert(meas[m].v2RootPos[0]-kpsLev[l][i].pt.x<eps && meas[m].v2RootPos[1]- kpsLev[l][i].pt.y<eps); // they shouldnt lie apart at all!

        BriefDataS* bData=new BriefDataS(m);
        if(mFeatureType==BRIEF_T_HIST)
          m->bData.push_back(bData); // save BRIEF data pointer in vector belonging to the MapPoint
        else{ // always update the descriptor
          if(m->bData.empty())
            m->bData.push_back(bData);
          else
            m->bData[0]=bData;
        }
        bData->v2p=meas[m].v2RootPos;
        bData->mpKF=&kF;
        // we are replacing an older BriefData entry by a newer one
        //if(it->first->pBData!=NULL) delete it->first->pBData; // not necessary now since we want to keep all BRIEF descriptors
        // TODO: store one descriptor per pyramid level?
        m->pBData=bData; // update latest BRIEF descriptor
        bData->descriptorFromCvMatRow(&bDesc.data[i*bDesc.cols]);
      }
      cout<<nCorners<<"/"<<kpsLev[l].size()<<" ";
      nBriefs+=kpsLev[l].size();
    }
    cout<<endl<< "#Corners="<<meas.size()<<" #BriefDescriptors="<<nBriefs<<" #NotExtracted="<<meas.size()-nBriefs<<" in dt="<<t0.toc()<<"ms"<<endl;

  }else if(mFeatureType==BRIEF_UNSCALED_T_HIST || mFeatureType==BRIEF_UNSCALED_T_UPDATE)
  {
    vector<vector<cv::KeyPoint> > kpsLev;
    convertToCvKeyPoints(meas,kpsLev,corresp);
    cout<<"Copied "<<kpsLev.size()<<" measurements for BRIEF descriptor extraction"<<endl;
    uint32_t nBriefs=0;
    cout<<"BRIEF single Scale: ";
    Timer t0; // take time
    for(size_t l=0; l<LEVELS;++l)
    {
      if(l>0)
        for(uint32_t i=0; i<kpsLev[l].size(); ++i)
        { // get the position in full size image.
          kpsLev[l][i].pt.x*=double((1<<l));
          kpsLev[l][i].pt.y*=double((1<<l));
        }

      cv::Mat bDesc;
      uint32_t nCorners=kpsLev[l].size();
      extractBRIEF(kpsLev[l],kF.aLevels[0].im,bDesc); // extract BRIEF descriptors at specific level l

      for(uint32_t i=0; i<kpsLev[l].size(); ++i)
      {
        MapPoint* m=corresp[kpsLev[l][i].class_id];
        assert(meas[m].v2RootPos[0]-kpsLev[l][i].pt.x<eps && meas[m].v2RootPos[1]- kpsLev[l][i].pt.y<eps); // they shouldnt lie apart at all!

        BriefDataS* bData=new BriefDataS(m);
        if(mFeatureType==BRIEF_UNSCALED_T_HIST)
          m->bData.push_back(bData); // save BRIEF data pointer in vector belonging to the MapPoint
        else{ // always update the descriptor
          if(m->bData.empty())
            m->bData.push_back(bData);
          else
            m->bData[0]=bData;
        }
        bData->v2p=meas[m].v2RootPos;
        bData->mpKF=&kF;
        // we are replacing an older BriefData entry by a newer one
        //if(it->first->pBData!=NULL) delete it->first->pBData; // not necessary now since we want to keep all BRIEF descriptors
        // TODO: store one descriptor per pyramid level?
        m->pBData=bData; // update latest BRIEF descriptor
        bData->descriptorFromCvMatRow(&bDesc.data[i*bDesc.cols]);
      }
      cout<<nCorners<<"/"<<kpsLev[l].size()<<" ";
      nBriefs+=kpsLev[l].size();
    }
    cout<<endl<< "#Corners="<<meas.size()<<" #BriefDescriptors="<<nBriefs<<" #NotExtracted="<<meas.size()-nBriefs<<" in dt="<<t0.toc()<<"ms"<<endl;

  }else if(mFeatureType==ORB_T_HIST || mFeatureType==FREAK_T_HIST ||
            mFeatureType==ORB_T_UPDATE || mFeatureType==FREAK_T_UPDATE)
  {
    cv::Mat bDesc;
    vector<cv::KeyPoint> kps;
    convertToCvKeyPoints(meas,kps,corresp);
    cout<<"Copied "<<kps.size()<<" measurements for ORB/FREAK descriptor extraction"<<endl;
    uint32_t nCorners=kps.size();
    extractBRIEF(kps,kF.aLevels[0].im,bDesc); // give it only the 0th pyramid level - pyramids are extracted internally.
    // kps have pyramid level/octave information stored within so ORB/FREAK knows where to extract
    for(uint32_t i=0; i<kps.size(); ++i)
    {
      MapPoint* m=corresp[kps[i].class_id];
      assert(meas[m].v2RootPos[0]-kps[i].pt.x<eps && meas[m].v2RootPos[1]- kps[i].pt.y<eps); // they shouldnt lie apart at all!

      BriefDataS* bData=new BriefDataS(m);

      if(mFeatureType==ORB_T_HIST || mFeatureType==FREAK_T_HIST){
        m->bData.push_back(bData);
        cout<<"appending feature"<< endl;
      }else{ // save ORB/FREAK data pointer in vector belonging to the MapPoint -> only store the most current
        if(m->bData.empty())
          m->bData.push_back(bData);
        else
          m->bData[0]=bData;
      }
      bData->v2p=meas[m].v2RootPos;
      bData->mpKF=&kF;
      // we are replacing an older BriefData entry by a newer one
      //if(it->first->pBData!=NULL) delete it->first->pBData; // not necessary now since we want to keep all BRIEF descriptors
      // TODO: store one descriptor per pyramid level?
      m->pBData=bData; // update latest BRIEF descriptor
      bData->descriptorFromCvMatRow(&bDesc.data[i*bDesc.cols]);
    }
    cout<<nCorners<<"/"<<kps.size()<<" ";
    cout<<endl<< "#Corners="<<meas.size()<<" #Descriptors="<<kps.size()<<" #NotExtracted="<<meas.size()-kps.size()<<endl;
  }
}

void BinaryExtractor::extractForFastCorners(KeyFrame &kF, vector<MapPoint*>& mps)
{
  // free all memory occupied by the previous run of this function TODO: good practice?
  BOOST_FOREACH(MapPoint* mp, mPFastCorner)
  {
    delete mp->pBData;
    delete mp->pTData;
    delete mp;
  }

  kF.MakeKeyFrame_Rest(); // does non-max suppression and Shi-Thomasi filtering
  // for FAST Corners and stores them in vCandidates.
  vector<cv::KeyPoint> kps;
  if(mFeatureType==BRIEF_T_HIST || mFeatureType==BRIEF_T_UPDATE)
  {
    for(size_t l=0; l<LEVELS;++l)
    { // have to do this because brief.compute deletes kps
      // entries if it cannot extract descriptors there.
      std::vector<Candidate>& meas=kF.aLevels[l].vCandidates;
      cout<<"Candidates @level "<<l<<": "<<meas.size()<<endl;
      convertToCvKeyPoints(meas,kps,l);
      cout<<"Copied "<<kps.size()<<" measurements for BRIEF descriptor extraction"<<endl;

      cv::Mat bDesc;
      extractBRIEF(kps,kF.aLevels[l].im,bDesc);
      cout << "------------------------------- BRIEF extraction done: #="<< bDesc.rows<<endl;

      // append BRIEF data to MapPoints which are returned
      int j=0, i=0;
      for (std::vector<Candidate>::iterator it=meas.begin(); it!=meas.end(); ++it,++i)
      {
        if(i==kps[j].class_id)
        { // have to do this because brief.compute deletes kps
          // entries if it cannot extract descriptors there.
          MapPoint* mp=new MapPoint();
          mps.push_back(mp);
          mPFastCorner.push_back(mp);
          BriefDataS* bData=new BriefDataS(mp);
          bData->v2p=it->v2RootPos;
          bData->mpKF=&kF;
          mp->pBData=bData;
          bData->descriptorFromCvMatRow(&bDesc.data[j*bDesc.cols]);
          TrackerData* tData=new TrackerData(mp);
          mp->pTData=tData;
          tData->nSearchLevel=l;
          tData->dSqrtInvNoise=1/(1<<l);
          tData->bFound=true;
          tData->v2Found=it->v2RootPos;
          ++j;
        }
      }
    }
  }else if(mFeatureType==BRIEF_UNSCALED_T_HIST || mFeatureType==BRIEF_UNSCALED_T_UPDATE)
  {
    for(size_t l=0; l<LEVELS;++l)
    { // have to do this because brief.compute deletes kps
      // entries if it cannot extract descriptors there.
      std::vector<Candidate>& meas=kF.aLevels[l].vCandidates;
      cout<<"Candidates @level "<<l<<": "<<meas.size()<<endl;
      convertToCvKeyPoints(meas,kps,l);
      cout<<"Copied "<<kps.size()<<" measurements for BRIEF descriptor extraction"<<endl;

      if(l>0)
        for(uint32_t i=0; i<kps.size(); ++i)
        { // get the position in full size image.
          kps[i].pt.x*=double((1<<l));
          kps[i].pt.y*=double((1<<l));
        }

      cv::Mat bDesc;
      extractBRIEF(kps,kF.aLevels[0].im,bDesc);
      cout << "------------------------------- BRIEF extraction done: #="<< bDesc.rows<<endl;

      // append BRIEF data to MapPoints which are returned
      int j=0, i=0;
      for (std::vector<Candidate>::iterator it=meas.begin(); it!=meas.end(); ++it,++i)
      {
        if(i==kps[j].class_id)
        { // have to do this because brief.compute deletes kps
          // entries if it cannot extract descriptors there.
          MapPoint* mp=new MapPoint();
          mps.push_back(mp);
          mPFastCorner.push_back(mp);
          BriefDataS* bData=new BriefDataS(mp);
          bData->v2p=it->v2RootPos;
          bData->mpKF=&kF;
          mp->pBData=bData;
          bData->descriptorFromCvMatRow(&bDesc.data[j*bDesc.cols]);
          TrackerData* tData=new TrackerData(mp);
          mp->pTData=tData;
          tData->nSearchLevel=l;
          tData->dSqrtInvNoise=1/(1<<l);
          tData->bFound=true;
          tData->v2Found=it->v2RootPos;
          ++j;
        }
      }
    }
  }else if(mFeatureType==ORB_T_HIST || mFeatureType==FREAK_T_HIST ||
            mFeatureType==ORB_T_UPDATE || mFeatureType==FREAK_T_UPDATE)
  {
    for(size_t l=0; l<LEVELS;++l)
    { // have to do this because brief.compute deletes kps
      // entries if it cannot extract descriptors there.
      std::vector<Candidate>& meas=kF.aLevels[l].vCandidates;
      cout<<"Candidates @level "<<l<<": "<<meas.size()<<endl;
      convertToCvKeyPoints(meas,kps,l,true);
      cout<<"Copied "<<kps.size()<<" measurements for ORB/FREAK descriptor extraction"<<endl;
    }
    cv::Mat bDesc;
    extractBRIEF(kps,kF.aLevels[0].im,bDesc);
    cout << "------------------------------- ORB/FREAK extraction done: #="<< bDesc.rows<<endl;

    // append BRIEF data to MapPoints which are returned
    for(size_t l=0; l<LEVELS;++l)
    {
      std::vector<Candidate>& meas=kF.aLevels[l].vCandidates;
      int j=0, i=0;
      for (std::vector<Candidate>::iterator it=meas.begin(); it!=meas.end(); ++it,++i)
      {
        if(i==kps[j].class_id)
        { // have to do this because brief.compute deletes kps
          // entries if it cannot extract descriptors there.
          MapPoint* mp=new MapPoint();
          mps.push_back(mp);
          mPFastCorner.push_back(mp);
          BriefDataS* bData=new BriefDataS(mp);
          bData->v2p=it->v2RootPos;
          bData->mpKF=&kF;
          mp->pBData=bData;
          bData->descriptorFromCvMatRow(&bDesc.data[j*bDesc.cols]);
          TrackerData* tData=new TrackerData(mp);
          mp->pTData=tData;
          tData->nSearchLevel=l;
          tData->dSqrtInvNoise=1/(1<<l);
          tData->bFound=true;
          tData->v2Found=it->v2RootPos;
          ++j;
        }
      }
    }
  }
}

