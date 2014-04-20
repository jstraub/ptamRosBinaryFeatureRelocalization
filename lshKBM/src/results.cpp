/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #include <results.hpp>

long int writeRelocResults(const vector<LocResult>& locRes,
    const vector<uint32_t>& frameIDs, string dir, long int timeTag, string tag)
{
  if(timeTag==0)
  {
    time_t rawtime; time(&rawtime);
    timeTag = rawtime;
  }
  char* buf = new char[dir.size()+tag.size()+100];
  if(tag.compare("noTag")==0)
    sprintf(buf,"%s/%ld_poses.txt",dir.c_str(),timeTag);
  else
    sprintf(buf,"%s/%ld_%s_poses.txt",dir.c_str(),timeTag,tag.c_str());
  cout<<"Writing relocalization results to "<<buf<<endl;
  ofstream outReLoc(buf);
  if(outReLoc)
  {
    for(uint32_t i=0;i<locRes.size(); ++i){
      outReLoc<<frameIDs[i]<<"\t"<<serializeForOutput(locRes[i].wTc)
            <<"\t"<<locRes[i].tReloc
            <<"\t"<<locRes[i].tPrep
            <<"\t"<<locRes[i].tPair
            <<"\t"<<locRes[i].querySize
            <<"\t"<<locRes[i].mapSize
            <<"\t"<<locRes[i].inliers<<endl;
    }
    outReLoc.close();
  }else{
    cout<< "Could not open "<<buf<<" for writing"<<endl;
  }
  if(tag.compare("noTag")==0)
    sprintf(buf,"%s/%ld_points3d.txt",dir.c_str(),timeTag);
  else
    sprintf(buf,"%s/%ld_%s_points3d.txt",dir.c_str(),timeTag,tag.c_str());
  ofstream outPoints(buf);
  if(outPoints)
  {
    cout<<"Writing points to "<<buf<<endl;
    for(uint32_t i=0;i<locRes.size(); ++i)
      for(uint32_t j=0;j<locRes[i].p3d.size(); ++j){
        outPoints<<frameIDs[i]<<"\t"<<locRes[i].p3d[j]<<endl;
      }
    outPoints.close();
  }else{
    cout<< "Could not open "<<buf<<" for writing"<<endl;
  }
  delete[] buf;
  return timeTag;
}

string serializeForOutput(SE3<double> pose)
{
  stringstream so;
  Vector<3>& trans=pose.get_translation();
  Matrix<3, 3> rot=pose.get_rotation().get_matrix(); // row major
  // output in column-major - for matlab! just use reshape(x,3,4)
  for (uint32_t j = 0; j < 3; ++j)
    for (uint32_t i = 0; i < 3; ++i)
      so << rot(i, j) << "\t";
  for (uint32_t i = 0; i < 3; ++i)
      so << trans[i] << "\t";
  return so.str();
}

