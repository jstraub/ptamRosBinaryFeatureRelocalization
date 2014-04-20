/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 
#include "Classifier.hpp"


double kCluster::silhouette(void) const
{
  //TODO: unbreak with new mClusters vector
//  // find labels for all datapoints
//  vector<uint32_t> l(k,0);
//  double J=evalCost(mCs);
//  cout<<" J="<<J<<endl;
//
//  vector<double> s(N,0.0); // s values for each datapoint
//  for(uint32_t i=0; i<N; ++i)
//  {
//    double a=0.0,b=0.0;
//    double Na=0.0,Nb=0.0;
//    for(uint32_t j=0; j<k; ++j)
//    {
//      double d=mMp[i]->pBData->distBrief(*(mCs[j]->pBData));
//      if(l[i]==j)
//      {// we are in the same cluster
//        a+=d;
//        ++Na;
//      }else { // we are in different clusters
//        b+=d;
//        ++Nb;
//      }
//    }
//    a/=Na; b/=Nb;
//    s[i]=a<b?(1-a/b):(b/a-1); // silhouette value for individual datapoints
//  }
//
//  vector<double> Sl(k,0.0); // silhouette value for clusters
//  vector<double> Nl(k,0.0); // number of datapoints in cluster
//  for(uint32_t i=0; i<N; ++i)
//  {
//    Sl[l[i]]+=s[i];
//    ++Nl[l[i]];
//  }
//  for(uint32_t i=0; i<k; ++i) // compute mean for each cluster
//    Sl[i]/=Nl[i];
//
//  double S=0.0;
//  for(uint32_t i=0; i<k; ++i) // compute overall mean
//    S+=Sl[i];
//
//  return S/double(k);
  return 0;
}

double kMedoids::cluster(uint32_t k_)
{
  k=k_; // how many clusters do we want?
  mCs.clear(); mCs.resize(k,0);
  for(uint32_t i=0; i<k; ++i) mCs[i]=mMp[mRnd(N)]; // initialize medoids randomly

  double J=0;
  while(42)
  {
    J=evalCost();
    cout<<"J="<<J<<endl;

    vector<MapPoint*> csij=mCs;
    // find neighboring set (to Imed) of medoids with the smallest cost function value
    double JijMin=FLT_MAX;
    vector<MapPoint*> csijMin;
    for(uint32_t i=0; i<k; ++i)
      for(uint32_t j=0; j<N; ++j)
      {
        csij[i]=mMp[j];
        vector<MapPoint*> csDummy=mCs; //stupid!!! only necessary because evalCost uses mCs by default - should be implemented differently, but kMedoids is not used anyway
        mCs=csij;
        double Jij=evalCost();
        csij=mCs;
        mCs=csDummy;
        cout<<"Jij="<<Jij<<" i="<<i<<" j="<<j<<endl;
        if(JijMin>Jij)
        {
          JijMin=Jij;
          csijMin=csij;
        }
      }
    if(JijMin<J)
    { // found medoids which have a smaller cost function
      mCs=csijMin;
      cout<<JijMin<<endl;
    }else break; // found (locally) optimal medoids
  }

  return J; // return score
}

double kMedoids::evalCost()
{
  uint32_t J=0;
  for (uint32_t j=0; j<N; ++j)
  {
    double dMin=FLT_MAX;
    for (uint32_t i=0; i<k; ++i)
    {
      uint32_t d=mMp[j]->pBData->distBrief(*(mCs[i]->pBData));
      if(dMin>d)
      {
        dMin=d;
      }
    }
    J+=dMin;
  }
  return double(J);
}

double kBinaryMeans::cluster(uint32_t k_)
{
// --------------------- Init
  k=k_; // how many clusters do we want?
  vector<MapPoint*> csPrev(k,NULL); // centroids from the previous iteration
  mCs.resize(k,NULL);
  mClusters.resize(k,vector<MapPoint*>());
  mCentroidsLUT.clear();

  cout<<"kBinaryMeans::cluster: N="<<N<<" mMp.size="<<mMp.size()<<endl;
  for(uint32_t i=0; i<k; ++i)
  {
    mCs[i]=new MapPoint();
    mCs[i]->pBData=new BriefData(mCs[i]);// initialize centroids randomly from datapoints
    uint32_t nRnd=mRnd.drawWithoutRepetition(N,i==0);
    mCs[i]->pBData->copyDataFrom(mMp[nRnd]->pBData);
    mCentroidsLUT[mCs[i]]=i;
//    cout<<nRnd<<" "<<mCs[i]->pBData->printForMatlab()<<endl;
    csPrev[i]=new MapPoint();
    csPrev[i]->pBData=new BriefData(csPrev[i]);
    csPrev[i]->pBData->copyDataFrom(mCs[i]->pBData);
  }
  cout<<"kBinaryMeans::cluster: Initialized centroids"<<endl;
// --------------------- EM to find k means
  double J=0;
  bool converged=false; // did the centroids change?
  uint32_t it=0;
  while(!converged)
  {
    // assign datapoints to the centroids mCs
    cout<<"kBinaryMeans::cluster: Evaluating cost function"<<endl;
    J=evalCost();
    cout<<" (Cost) J="<<J<<" @iteration="<<it<<endl;
    // recompute means
    for(uint32_t i=0; i<k; ++i)
    {
      vector<uint32_t> score(BriefData::BRIEF_K*8,0);
      uint32_t scoreMax=mClusters[i].size();
      for(uint32_t j=0; j<mClusters[i].size(); ++j)
      {// basically voting on bit positions in the BriefDescriptor form all descriptors in the cluster
        mClusters[i][j]->pBData->incrBriefScore(score);
      }
      // obtain new cluster center from the voting on the bit positions -> new mean in the binary vectorspace
//      cout<<" scoreMax="<<scoreMax<<endl;
//      for(uint32_t j=0; j<BriefData::BRIEF_K*8; ++j)
//        cout<<score[j]<<"\t";
//      cout<<endl;
      mCs[i]->pBData->descriptorFromBriefScore(score, scoreMax);
    }

    // check whether means have changed
    converged=true;
    for(uint32_t i=0; i<k; ++i)
    {
      uint32_t d=csPrev[i]->pBData->distBrief(*(mCs[i]->pBData));
//      cout<<"d("<<i<<")="<<d<<":"<<endl;
//      cout<<*(csPrev[i]->pBData)<<endl;
//      cout<<*(cs[i]->pBData)<<endl;
      if(d > 0)
      {// if distance is not 0 the centroid has changed
        converged=false;
        cout<<"d("<<i<<")="<<d<<": not converged! "<<endl;
        break;
      }
    }
    // remember centroids
    for(uint32_t i=0; i<k; ++i){
      csPrev[i]->pBData->copyDataFrom(mCs[i]->pBData);
      //cout<<mCs[i]->pBData->printForMatlab()<<endl;
    }
    ++it;
  }
  cout<<"final J="<<J<<endl;

  for(uint32_t i=0; i<k; ++i)
  {
    delete csPrev[i]->pBData;
    delete csPrev[i];
  }

  J=evalCost();
  return J; // return score
}

double kBinaryMeans::evalCost()
{
  uint32_t J=0;
  for (uint32_t i=0; i<k; ++i)
  {
    mClusters[i].clear();
  }
//  cout<<"mCluster.size()="<<mClusters.size()<<" mMp.size()="<<mMp.size()<<endl;
  for (uint32_t j=0; j<N; ++j)
  {
    uint32_t dMin=INT_MAX;
    uint32_t iMin=0;
    for (uint32_t i=0; i<k; ++i)
    {
      uint32_t d=mMp[j]->pBData->distBrief(*(mCs[i]->pBData));
      if(dMin>d){
        dMin=d;
        iMin=i;
      }
    }
    mClusters[iMin].push_back(mMp[j]);
    J+=dMin;
  }

  return double(J)/double(N);
}

float NearestNeighbor::pair(vector<MapPoint*>::const_iterator& queryMPBegin,
    vector<MapPoint*>::const_iterator& queryMPEnd,
    vector<BriefAssociation>::iterator& pairingBegin,
    vector<BriefAssociation>::iterator& pairingEnd)
{
  vector<BriefAssociation>::iterator itPair=pairingBegin;
  Timer t0;
  for(vector<MapPoint*>::const_iterator itQuery=queryMPBegin; itQuery!=queryMPEnd; itQuery++, itPair++)
  {
    assert(itQuery!=queryMPEnd);
    assert(itPair!=pairingEnd);
    this->pair(*itQuery,*itPair);
  };
  return t0.toc();
}

float NearestNeighbor::pair(vector<MapPoint*>& queryMPs,
    vector<BriefAssociation>& pairing)
{
  pairing.resize(queryMPs.size(),BriefAssociation(INT_MAX,NULL,NULL));
  size_t i=0;
  Timer t0;
  BOOST_FOREACH(MapPoint* queryMP, queryMPs)
  {
    this->pair(queryMP,pairing[i]);
    ++i;
  }
  t0.toc();

#ifdef _DEBUG
  cout<<"NN pairing: sorting "<<pairing.size()<<" pairings - matched "<<queryMPs.size()
      <<" current features against "<< mMPs.size()<<" MapPoints in the Map. ("
      <<nullCount<<" NULL BriefDatas)"<<endl;
  std::sort(pairing.begin(),pairing.end()); // TODO: maybe there is faster sorting if we only need smallest?
  size_t nPairings=0;
  // output distances
  for(size_t i=0; i<pairing.size();++i)
    if(pairing[i].mp2!=NULL)
    {
//      if(pairing[i].dist<10)
//        cout<<(*nnPairing[i].mp1->pBData)<<" to "<<*(nnPairing[i].mp2->pBData)<<" = "<<
      cout<<pairing[i].dist <<"; ";
      nPairings++;
    }
  cout<<" #="<< nPairings<<" ("<<pairing.size()-nPairings <<" not paired!)"<<endl;
#endif
  return t0.lastDt();
}

float LSH::prepare(void)
{
  // if no parameters are set, we come up with our own params (from paper)
//  if(ml==0 || mk==0)
//  {
//    const double B=12.0; // max number of features in bins
//    const double n=mMPs.size(); // number of elemets for hastable
//    const double p1=0.95; // p1 is the probability that if datapoints are close that they land in the same bucket
//    const double p2=0.05; // p2 is the probability that if datapoints are apart that they land in the different bucket
//    const double sig=log(1/p1)/log(1/p2);
//    ml=ceil(pow(n/B,sig)); // number of hashtables
//    mk=ceil(log(n/B)/log(1/p2)); // bit length of hash function
//  }

  Random rnd(time(NULL));
  // erase everything since params might have changed
  if(mLshTables.size()!=0) mLshTables.clear();
  if(mBitChangeProb.size()==0)
  {
//    cout<<"Hash function: bit positions sampled with uniform probability density"<<endl;
    for(uint32_t i=0; i<ml; ++i)
    {
      mLshTables.push_back(new LshTable(rnd,mk));
//      vector<uint32_t> g=mLshTables[i]->getHashFunction();
//      cout<<"Hash function "<<mk<<":"<<endl;
//      BOOST_FOREACH(uint32_t gi, g)
//      {
//        cout<<gi<<" ";
//      }; cout<<endl;
    }
  }else{
//    cout<<"Hash function: bit positions sampled from bit change probability"<<endl;
    for(uint32_t i=0; i<ml; ++i)
    {
      mLshTables.push_back(new LshTable(rnd,mk,mBitChangeProb));
//      vector<uint32_t> g=mLshTables[i]->getHashFunction();
//      cout<<"Hash function "<<mk<<":"<<endl;
//      BOOST_FOREACH(uint32_t gi, g)
//      {
//        cout<<gi<<" ";
//      }; cout<<endl;
    }
  }

//  cout << "LSH with "<<ml<< " hash-tables from "<<mMPs.size()
//      <<" features. Hash function looks at "<<mk<<" bits of the BRIEF descriptor."<<endl;

  Timer t0;
  uint32_t notAdded=0;
  bool once=false;
  BOOST_FOREACH(MapPoint* mp, mMPs)
  {
    if(mp != NULL && mp->pBData != NULL)
    {
      for(uint32_t i=0; i<ml; ++i)
      { // fill hash-tables
        //      cout<<"storing "<<mp<<" "<<mp->pBData<<" in "<<mLshTables[i]<<endl;
        mLshTables[i]->store(mp);
      }
    }else{
      if(mp != NULL && mp->pBData == NULL)
      {
        if(!once){ cout<<"KeyframeIds which have MapPoints with empty BriefData: "; once=true;}
        cout<<mp->pPatchSourceKF->mSeq<<" ";
      }
      ++notAdded;
    }
    //    cout<<"notAdded="<<notAdded<<" ";
  }
  if(once) cout<<endl;
  t0.toc();
//  cout<<endl<<"========== Build Tables in "<<t0<<endl;
  return t0.lastDt();
}

float LSH::pair(vector<MapPoint*>::const_iterator& queryMPBegin,
    vector<MapPoint*>::const_iterator& queryMPEnd,
    vector<BriefAssociation>::iterator& pairingBegin,
    vector<BriefAssociation>::iterator& pairingEnd)
{
  vector<BriefAssociation>::iterator itPair=pairingBegin;
  Timer t0;
  for(vector<MapPoint*>::const_iterator itQuery=queryMPBegin; itQuery!=queryMPEnd; itQuery++, itPair++)
  {
    this->pair(*itQuery,*itPair);
  }
  return t0.toc();
}

float LSH::pair(vector<MapPoint*>& queryMPs,
        vector<BriefAssociation>& pairing, vector<uint32_t>& nRetrieved)
{
  pairing.resize(queryMPs.size(),BriefAssociation(INT_MAX,NULL,NULL));
  nRetrieved.resize(queryMPs.size(),0);

  Timer t0; t0.tic();
  // find nearest neighbor
  size_t i=0;
//  cout<<"LSH pair query#="<<queryMPs.size()<<endl;
  BOOST_FOREACH(MapPoint* mp, queryMPs)
  {
    pair(mp,pairing[i],nRetrieved[i]);
    ++i;
  }
  t0.toc();
//  cout<<"========== Finding LSH Neighbours dt="<<t0<<" querryied "<< queryMPs.size()<<" in ";
//  for(uint32_t i=0; i<ml; ++i)
//    cout<<mLshTables[i]->numElems()<<" & ";
//  cout<<" map features."<<endl;

  return t0.lastDt();
}

void LSH::pair(MapPoint* queryMP, BriefAssociation& annPair, uint32_t& nRetrieved)
{
//  map<MapPoint*,bool> candMap;
//  uint32_t nAllCandidates=0;
//  for(uint32_t i=0; i<ml; ++i)
//  {
//    vector<MapPoint*> c=mLshTables[i]->get(queryMP);
//    nAllCandidates+=c.size();
//    for(uint32_t j=0; j<c.size(); ++j)
//      candMap[c[j]]=true;
//  }
//  vector<MapPoint*> candidates; candidates.reserve(candMap.size());
//  for(map<MapPoint*,bool>::iterator it=candMap.begin(); it!=candMap.end(); it++)
  //    candidates.push_back(it->first);
  vector<MapPoint*> candAll;
  for(uint32_t i=0; i<ml; ++i)
  {
    const vector<MapPoint*>& c=mLshTables[i]->get(queryMP);
    candAll.reserve(candAll.size()+c.size());
    for(uint32_t j=0; j<c.size(); ++j)
      candAll.push_back(c[j]);
  }
  sort(candAll.begin(), candAll.end()); // sort according to pointer location to find duplicates
  vector<MapPoint*> candidates; candidates.reserve(candAll.size());
  MapPoint* prevP=NULL;
  for(uint32_t i=0; i<candAll.size(); ++i)
    if(candAll[i] != prevP)
    {
      candidates.push_back(candAll[i]);
      prevP=candAll[i];
    }

  nRetrieved=candidates.size();
  if(nRetrieved>0){
    //    cout<<candidates.size()<<endl;
    NearestNeighbor NN(candidates);
    NN.pair(queryMP, annPair);
  }else{
    annPair=BriefAssociation(INT_MAX,queryMP,NULL);
  }
//  cout<<double(candidates.size())/candAll.size()<<" ";
//  cout<<"nAllCandidates="<<nAllCandidates<<" candidatesForNN="<<candidates.size()<<endl;
}

float kLSH::prepare(void)
{
  Random rnd(time(NULL));
  // erase everything since params might have changed
  if(mLshTables.size()!=0) mLshTables.clear();
  if(mBitChangeProb.size()==0)
  {
    cout<<"Hash function: bit positions sampled with uniform probability density"<<endl;
    for(uint32_t i=0; i<ml; ++i)
    {
      mLshTables.push_back(new LshTable(rnd,mm));
      vector<uint32_t> g=mLshTables[i]->getHashFunction();
//      cout<<"Hash function "<<mk<<":"<<endl;
//      BOOST_FOREACH(uint32_t gi, g)
//      {
//        cout<<gi<<" ";
//      }; cout<<endl;
    }
  }else{
    cout<<"Hash function: bit positions sampled from bit change probability"<<endl;
    for(uint32_t i=0; i<ml; ++i)
    {
      mLshTables.push_back(new LshTable(rnd,mm,mBitChangeProb));
      vector<uint32_t> g=mLshTables[i]->getHashFunction();
//      cout<<"Hash function "<<mk<<":"<<endl;
//      BOOST_FOREACH(uint32_t gi, g)
//      {
//        cout<<gi<<" ";
//      }; cout<<endl;
    }
  }

  cout << "kLSH with "<<ml<< " hash-tables from "<<mMPs.size()
      <<" features. Hash function looks at "<<mm<<" bits of the BRIEF descriptor."<<endl;
  // fill hash-tables
  Timer t0;
  for(uint32_t i=0; i<ml; ++i)
  {
    uint32_t notAdded=0;
    BOOST_FOREACH(MapPoint* mp, mMPs)
    {
//      cout<<"storing "<<mp<<" "<<mp->pBData<<" in "<<mLshTables[i]<<endl;
      if(mp != NULL && mp->pBData != NULL)
        mLshTables[i]->store(mp);
      else{
        if(mp != NULL)
        {
          cout<<mp->pPatchSourceKF->mSeq<<" ";
        }
        ++notAdded;
      }
    }
//    cout<<"notAdded="<<notAdded<<" ";
  }
  t0.toc();
  cout<<endl<<"========== Build Tables in "<<t0<<endl;
  return t0.lastDt();
}

float kLSH::pair(vector<MapPoint*>& queryMPs,
        vector<BriefAssociation>& pairing, vector<uint32_t>& nRetrieved)
{
  pairing.clear();
  pairing.reserve(queryMPs.size()*mk);
  nRetrieved.clear();
  nRetrieved.resize(queryMPs.size(),0);

  Timer t0; t0.tic();
  // find nearest neighbor
  size_t ind=0;
  cout<<"LSH pair query#="<<queryMPs.size()<<endl;
  BOOST_FOREACH(MapPoint* mp, queryMPs)
  {
    vector<BriefAssociation> knn;
    pair(mp,knn,nRetrieved[ind]);
    for(uint32_t i=0; i<mk; ++i) pairing.push_back(knn[i]); // k Nearest neighbors
    ++ind;
  }
  t0.toc();
  cout<<"========== Finding LSH Neighbours dt="<<t0<<" querryied "<< queryMPs.size()<<" in ";
  for(uint32_t i=0; i<ml; ++i)
    cout<<mLshTables[i]->numElems()<<" & ";
  cout<<" map features."<<endl;

  return t0.lastDt();
}

void kLSH::pair(MapPoint* queryMP, vector<BriefAssociation>& annPair, uint32_t& nRetrieved)
{
  annPair.clear();
  vector<MapPoint*> candidates;
  for(uint32_t i=0; i<ml; ++i)
  {
    vector<MapPoint*> c=mLshTables[i]->get(queryMP);
    for(uint32_t j=0; j<c.size(); ++j)
      candidates.push_back(c[j]);
  }
  nRetrieved=candidates.size();
  if(nRetrieved>0){
    NearestNeighbor NN(candidates);
    NN.pair(queryMP, annPair,mk);
  }
}
