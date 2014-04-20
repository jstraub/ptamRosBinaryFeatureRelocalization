/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #include "helpers.hpp"


void compairAgainstNN(const vector<BriefAssociation>& pairing,
    const vector<BriefAssociation>& pairingNN,
    uint32_t& tp)
{// compare LSH pairing against NN (ground truth?)
//  cout<<"compare LSH pairing against NN (ground truth?)"<<endl;
  tp=0;
  for(size_t i=0; i<pairingNN.size();++i)
  {
    assert(pairingNN[i].mp2!=NULL);
    for(size_t j=0; j<pairing.size();++j)
      if((pairingNN[i].mp1==pairing[j].mp1) && (pairingNN[i].mp2==pairing[j].mp2))
      {
        ++tp;
        break;
      }
  }
  cerr<<endl<<"tp="<<tp<<" pairingNN.size()="<<pairingNN.size()<<" pairing.size()="<<pairing.size()<<endl;
//  cout<<"#LSH matched equal to NN="<<nEqual<<" #LSH matched differently from NN="<<nDiff<< " (total#="<<pairingNN.size()<<")"<<endl;
}

void evalPairing(const vector<BriefAssociation>& pairing, double& avgDist,
    uint32_t& notPaired)
{
  notPaired=0;
  double sumDist=0, N=0;
  for(size_t i=0; i<pairing.size();++i)
    if(pairing[i].mp2!=NULL){
      sumDist+=pairing[i].dist;
      ++N;
    }else{
      ++notPaired;
    }
  cout<<"Mean Brief to Brief Distance="<<sumDist/N
      <<" # not paired="<<notPaired<<endl;
  avgDist=sumDist/N;
}


