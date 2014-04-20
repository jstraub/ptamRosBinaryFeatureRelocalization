/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #include <BriefData.hpp>

bool loadBriefs(const std::string pathToData,
    const std::string fileName,
    std::vector<BriefData*>& briefs,
    std::map<BriefData*,size_t>& briefFrames)
{
  char buf[fileName.size()+10+pathToData.size()];
  {// load brief data from files
    sprintf(buf,"%s/%s",pathToData.c_str(),fileName.c_str());
    std::ifstream in(buf);
    if(!in.is_open()) return false;
    double w_x;
    while(in>>w_x)
    {
      MapPoint* mp=new MapPoint();
      BriefDataS* brief=new BriefDataS(mp);
      int32_t frameNr=-1;
//      mp->pBData=brief;
      mp->v3WorldPos[0]=w_x;
      in>>mp->v3WorldPos[1]>>mp->v3WorldPos[2]>>frameNr>>brief->v2p[0]>>brief->v2p[1];
      for (size_t j=0; j<BriefData::BRIEF_K/4; ++j)
      {
        uint32_t d; in>>d;
        brief->bd[j*4]=uint8_t(d>>24);
        brief->bd[j*4+1]=uint8_t(d>>16);
        brief->bd[j*4+2]=uint8_t(d>>8);
        brief->bd[j*4+3]=uint8_t(d);
      }
      briefs.push_back(brief);
      briefFrames[brief]=frameNr;
    }
  }
  return true;
}

bool loadHistoryBriefs(const string pathToData,
    const string fileName,
    vector<vector<BriefData*> >& briefs,
    map<BriefData*,size_t>& briefFrames)
{
  char buf[fileName.size()+10+pathToData.size()];
  {// load brief data from files
    sprintf(buf,"%s/%s",pathToData.c_str(),fileName.c_str());
    std::ifstream in(buf);
    if(!in.is_open()) return false;
    briefs.clear(); briefs.resize(1,vector<BriefData*>());
    uint32_t i=0;
    uint32_t id=0,idPrev=id;
    while(in>>id)
    {
      MapPoint* mp=new MapPoint();
      BriefDataS* brief=new BriefDataS(mp);
      int32_t frameNr=-1;
      in>>mp->v3WorldPos[0]>>mp->v3WorldPos[1]>>mp->v3WorldPos[2]>>frameNr>>brief->v2p[0]>>brief->v2p[1];
      for (size_t j=0; j<BriefData::BRIEF_K/4; ++j){
        uint32_t d;
        in>>d;
        brief->bd[j*4]=uint8_t(d>>24);
        brief->bd[j*4+1]=uint8_t(d>>16);
        brief->bd[j*4+2]=uint8_t(d>>8);
        brief->bd[j*4+3]=uint8_t(d);
      }
      briefFrames[brief]=frameNr;

      if(id==idPrev)
      { // we already opened a new bucket for the BRIEF descriptors of this MapPoint
        briefs[i].push_back(brief);
      }else{ // open a new bucket for BRIEF descriptors of new MapPoint
        briefs.push_back(std::vector<BriefData*>(1,brief));
        ++i;
      }
      idPrev=id;
    }
  }
  return true;
}

// used in testLSHRansacLoc.cpp to allow testing of full history vs updating
bool loadHistoryBriefs(const string pathToData,
    const string fileName,
    std::map<BriefData*,uint32_t>& briefFrames,
        std::map<uint32_t,vector<MapPoint*> >& frames)
{
  char buf[fileName.size()+10+pathToData.size()];
  {// load brief data from files
    sprintf(buf,"%s/%s",pathToData.c_str(),fileName.c_str());
    std::ifstream in(buf);
    if(!in.is_open()) return false;
    MapPoint* mpPrev=NULL;
    uint32_t id=0,idPrev=99999;
    while(in>>id)
    {
      MapPoint* mp=NULL;
      int32_t frameNr=-1;
      double x,y,z;
      in>>x>>y>>z>>frameNr;
      if(id==idPrev)
      { // we already opened a new bucket for the BRIEF descriptors of this MapPoint
        mp=mpPrev;
      }else{ // open a new bucket for BRIEF descriptors of new MapPoint
        mp=new MapPoint();
        mp->v3WorldPos[0]=x; // only need to assign them once since the d pos remains the same and equal to the last estimate of the 3D pos
        mp->v3WorldPos[1]=y;
        mp->v3WorldPos[2]=z;
      }
      frames[frameNr].push_back(mp);
      BriefDataS* brief=new BriefDataS(mp);
      in>>brief->v2p[0]>>brief->v2p[1];
      for (size_t j=0; j<BriefData::BRIEF_K/4; ++j){
        uint32_t d;
        in>>d;
        brief->bd[j*4]=uint8_t(d>>24);
        brief->bd[j*4+1]=uint8_t(d>>16);
        brief->bd[j*4+2]=uint8_t(d>>8);
        brief->bd[j*4+3]=uint8_t(d);
      }
      brief->mpKF=(KeyFrame*)frameNr; // abuse the pointer for storing the number of the keyframe so we can later filter for the keyframe nr.
//      cout<<brief->mpKF<<" ";
      briefFrames[brief]=frameNr;
      mp->bData.push_back(brief); // build history -> latest observation will be last element to be pushed onto bData due to ordering in allHistoryKeyBriefDescriptors.txt

      idPrev=id;
      mpPrev=mp;
    }
  }
  return true;
}

bool loadFrameBriefs(const std::string pathToData,
    const std::string fileName,
    std::map<BriefData*,uint32_t>& briefFrames,
    std::map<uint32_t,vector<BriefData*> >& frames)
{
  char buf[fileName.size()+10+pathToData.size()];
  {// load brief data from files
    sprintf(buf,"%s/%s",pathToData.c_str(),fileName.c_str());
    std::ifstream in(buf);
    if(!in.is_open()) return false;
    uint32_t id=0;
    while(in>>id)
    {
      MapPoint* mp=new MapPoint();
      BriefDataS* brief=new BriefDataS(mp);
      int32_t frameNr=-1;
      in>>mp->v3WorldPos[0]>>mp->v3WorldPos[1]>>mp->v3WorldPos[2]>>frameNr>>brief->v2p[0]>>brief->v2p[1];
      for (size_t j=0; j<BriefData::BRIEF_K/4; ++j){
        uint32_t d;
        in>>d;
        brief->bd[j*4]=uint8_t(d>>24);
        brief->bd[j*4+1]=uint8_t(d>>16);
        brief->bd[j*4+2]=uint8_t(d>>8);
        brief->bd[j*4+3]=uint8_t(d);
      }
      briefFrames[brief]=frameNr;
      frames[frameNr].push_back(brief);
    }
  }
  return true;
}


//std::ostream& operator<< (std::ostream &out, const BriefData& a){
//  out<<"[";
//  for (size_t i=0; i<BriefDesc::BRIEF_K; ++i)
//  {
//    for(int j=0; j<8;++j)
//      out<<((a.bd[i]>>j) & 1);
//    out<<".";
//  }
//  out<<"]BD";
//  return out;
//};
//
//std::ostream& operator<< (std::ostream &out, const BriefDataM& a){
//  out<<"[";
//  for (size_t i=0; i<BriefDesc::BRIEF_K; ++i)
//  {
//    for(int j=0; j<8;++j)
//      out<<((a.bd[i]>>j) & 1);
//    out<<".";
//  }
//  out<<"]BD";
//  return out;
//};

//
//// load BRIEF descriptors of keypoints seen in keyFrames
//bool loadKeyFrameNrs(const std::string pathToData,
//    std::vector<size_t>& keyFrameNrs)
//{
//  char buf[60+pathToData.size()];
//  {// read key Frame numbers from file
//    sprintf(buf,"%s/keyFrameIDs.txt",pathToData.c_str());
//    std::ifstream in(buf);
//    size_t frameNr;
//    cerr<<"KeyFrame ids: ";
//    while(in>>frameNr)
//    {
//      keyFrameNrs.push_back(frameNr);
//      cerr<<frameNr<<"\t";
//    }
//    cerr<<endl;
//  }
//  if(keyFrameNrs.size()<1)
//    return false; // there should always be at least 2 keyframes.
//  else
//    return true;
//}
