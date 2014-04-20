/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #include <lshKBM.hpp>

void loadLshKBM(FeatureDatabase& fdb, LshKBM& lshKBM,
    vector<uint32_t>& imageIds, string pathToLshKBM)
{ // load/create quantizer
  FILE *fTest=fopen(pathToLshKBM.c_str(),"rb");
  cout<<pathToLshKBM<<endl;
  if(!fTest){
    cout<<"No DB found at"<<pathToLshKBM<<" -> building!"<<endl;
    Timer t0;
    lshKBM.create(fdb,imageIds);
    t0.toc();
    cout<<" ----- dT for visual word DB creation: "<<t0<<endl;
    FILE *fOut=fopen(pathToLshKBM.c_str(),"wb");
    cout<<"Saving DB at "<<pathToLshKBM<<endl;
    lshKBM.save(fOut,0);
    fclose(fOut);
  }else{
    lshKBM.load(fTest);
    fclose(fTest);
  }
}

//TODO:  I do NOT set the image positions or 3D positions anymore!!!
void LshKBM::create(const FeatureDatabase& db,
    std::vector<unsigned int> image_ids)
{
  generateGUID();
  uint32_t dims = db.dims(); // number of bytes per feature descriptor
  assert(dims==BriefDesc::BRIEF_K);// BRIEF32 features

  Timer t0;
  const db_index_t& dbIndex=db.index(); // for each image
  vector<BriefDesc*> mapDs;
  mapDs.reserve(db.descCount()); // reserve storage for all descriptors

  for(uint32_t j=0; j<image_ids.size(); ++j)
  { // iterate over images in image_ids
    uint32_t i=image_ids[j];
    uint8_t* desc = ((uint8_t*)db.firstDescriptor(i)); // for each feature
//    const meta_file_entry* dbMeta=db.firstMetaEntry(i); // for each feature
    for(int j=0; j<dbIndex[i].desc_count; ++j)
    {// iterate over the features found in image with index i
      // dont copy data just set pointer to the data
       BriefDescM* bD = new BriefDescM(&desc[j*dims]);
      mapDs.push_back(bD);
    }
  }
  cout<<" dt for loading: "<<t0.toc()<<"ms"<<endl;

  num_images=image_ids.size();
  if(mLshKbmP.getMaxSamples()>0)
  {
    cout<<"LshKBM::create: Randomly subsampling "<<mapDs.size()<<" to "<<mLshKbmP.getMaxSamples()<<endl;
    assert(mLshKbmP.getMaxSamples()<=mapDs.size());
    num_descriptors=mLshKbmP.getMaxSamples();
    vector<BriefDesc*> mapDSamples(mLshKbmP.getMaxSamples());
    __gnu_cxx::random_sample(mapDs.begin(),mapDs.end(),mapDSamples.begin(),mapDSamples.end());
    cout<<"LshKBM::create: random subsampling done!"<<endl;
    t0.tic();
    prepare(mapDSamples);
    t0.toc();
  }else{
    num_descriptors=mapDs.size();
    t0.tic();
    prepare(mapDs);
    t0.toc();
  }
  mDtKBinMeans=t0.lastDt();
}

void LshKBM::quantizeForRegistration(QuantizerCallback& cb,
    BriefDesc* query,
    QuantizerContext& ctxt) const
{
  vector<Assoc<BriefDesc,uint32_t> > annPairs;

  // it is possible, that two or more matches have the same Hamming distance.
#ifdef NN_REGISTRATION
  NearestNeighbor<BriefDesc,uint32_t> nn(mCentroids);
  nn.pair(query,annPairs);
#else
  uint32_t trials=mpLSH->pairAllways(query,annPairs);
#endif

  for(uint32_t i=0; i<annPairs.size(); ++i)
  { // register all pairings with the same hamming distance
    assert(annPairs[i].m!=NULL);
    uint32_t visualWord=mVisualWordLUT.find(annPairs[i].m)->second;
    cb.quantizedForRegistrationCallback(visualWord, ctxt, true); //normalisiert weil true
  }
}

void LshKBM::quantizeForRegistration(QuantizerCallback& cb,
    const CvMat *descriptor,
    QuantizerContext& ctxt) const
{
  assert(type==CV_8UC1);
  BriefDescM query((uint8_t*)(descriptor->data.ptr));
  quantizeForRegistration(cb, &query, ctxt);
}

void LshKBM::quantizeForQuery(QuantizerCallback& cb,
    BriefDesc* query,
    QuantizerContext& ctxt) const
{
  vector<Assoc<BriefDesc,uint32_t> > annPairs;
  uint32_t trials=mpLSH->pairAllways(query,annPairs);
  for(uint32_t i=0; i<annPairs.size(); ++i)
  {
    assert(annPairs[i].m!=NULL);
    uint32_t visualWord=mVisualWordLUT.find(annPairs[i].m)->second;
    cb.quantizedForQueryCallback(visualWord, ctxt, 1.0);
  }
//  if(trials>0) cout<<"("<<trials<<" "<<annPairs[0].d<<") ";
}

void LshKBM::quantizeForQuery(QuantizerCallback& cb,
    const CvMat *descriptor,
    QuantizerContext& ctxt,
    bool estimate_P_correct_quant) const
{
  assert(type==CV_8UC1);
  BriefDescM query((uint8_t*)descriptor->data.ptr);
  quantizeForQuery(cb, &query, ctxt);
}

bool LshKBM::save(FILE *f, unsigned char ver) const
{
  unsigned int signature = 0xFAADBEEB;
  WRITE_VAR(signature);
  ver = mVer; // added mJfinal and mDtKBinMeans
  WRITE_VAR(ver);
  WRITE_VAR(type);
  WRITE_VAR(dims);
  WRITE_VAR(num_descriptors);
  WRITE_VAR(num_images);
  unsigned int word_count = wordCount(); // how many centroids?
  WRITE_VAR(word_count);
  WRITE_VAR(mJfinal);
  WRITE_VAR(mDtKBinMeans);
  WRITE_VAR(m_guid);
  uint32_t m=mLshKbmP.getM(),l=mLshKbmP.getL();
  WRITE_VAR(m);
  WRITE_VAR(l);

  cout<<"sig="<<signature<<" ver="<<ver<<" type="<<type<<" dims="<<dims<<" #descriptors="<<num_descriptors<<" #images="<<num_images<<" #words="<<word_count<<endl;
  cout<<" mJfinal="<<mJfinal<<" mDtKBinMeans="<<mDtKBinMeans<<endl;

  // write centroids to file - decide which one to save!
  if(mCentroids.size()>0){
    for(uint32_t i=0; i<mCentroids.size(); ++i)
      WRITE_PTR(&(mCentroids[i]->bd[0]), dims*sizeof(mCentroids[i]->bd[0]));
  }else{
    Log::error("Could not determine which centroids to save to file!");
    return false;
  }
  //TODO: maybe save LSH buckets here? try without first
  // LSH table building should be quite fast.
  return true;
}

Quantizer* LshKBM::load(FILE *f)
{
  cout<<"LshKBM::load: loading"<<endl;
  unsigned int signature=0;
  unsigned char ver=99;

  READ_VAR(signature); assert(signature==0xFAADBEEB);
  READ_VAR(ver); assert(ver<=mVer);
  mVer=ver;
  READ_VAR(type); assert(type==CV_8UC1);
  READ_VAR(dims); assert(dims==BriefDesc::BRIEF_K);
  READ_VAR(num_descriptors);
  READ_VAR(num_images);
  unsigned int word_count;
  READ_VAR(word_count);
  // Create forest of kd-trees for searching
  switch(mVer) {
  case 0:
    mJfinal=256.0;
    mDtKBinMeans=0.0;
    for(uint32_t i=0;i<guidLen+1;++i)
      m_guid[i]=0;
    break;
  case 1:
    READ_VAR(mJfinal);
    READ_VAR(mDtKBinMeans);
    for(uint32_t i=0;i<guidLen+1;++i)
      m_guid[i]=0;
    break;
  case 2:
    READ_VAR(mJfinal);
    READ_VAR(mDtKBinMeans);
    READ_VAR(m_guid);
    break;
  case 3:
    READ_VAR(mJfinal);
    READ_VAR(mDtKBinMeans);
    READ_VAR(m_guid);
    uint32_t m,l;
    READ_VAR(m);
    READ_VAR(l);
    mLshKbmP=LshKBMParams(word_count,l,m,mLshKbmP.getThreadMax());
    break;
  default:
    Log::error("Invalid file version: %d!\n", ver);
    return NULL;
  }

//  m_num_clusters = word_count;
  cout<<"sig="<<signature<<" ver="<<ver<<" type="<<type<<" dims="<<dims<<" #descriptors="<<num_descriptors<<" #images="<<num_images<<" #words="<<word_count<<endl;
  cout<<" mJfinal="<<mJfinal<<" mDtKBinMeans="<<mDtKBinMeans<<endl;

  mLshKbmP=LshKBMParams(word_count,mLshKbmP.getL(),mLshKbmP.getM(),mLshKbmP.getThreadMax());
  mCentroids.resize(mLshKbmP.getK(), NULL);
  for(uint32_t i=0; i<mLshKbmP.getK(); ++i)
  { // create all centroids
    mCentroids[i]=new BriefDescS();
    READ_PTR(&(mCentroids[i]->bd[0]), dims*sizeof(mCentroids[i]->bd[0]));
  }

  // Create forest of kd-trees for searching
  switch(ver) {
  case 0:
  case 1:
  case 2:
  case 3:
    Log::info("Quantizer file had NO prebuilt LSH\n");
    // build hash tables - the class will realize,
    // that the centroids have been loaded already!
    prepareFromCentroids(mCentroids);
    cout<<"LshKBM::load: loaded successfully"<<endl;
    break;
  default:
    Log::error("Invalid file version: %d!\n", ver);
    return NULL;
  }
  return this;
}
void LshKBM::dump(int stats_only) const
{
  cout<<"dumping LshKBM"<<endl;
}

