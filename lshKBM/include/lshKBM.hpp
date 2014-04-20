 
#pragma once

#include <db.h>
#include <log.h>
#include <BriefDesc.hpp>
#include <lsh.hpp>
#include <nn.hpp>
#include <timer.hpp>
#include <kbm.hpp>
#include <quantizer.h>

#include <opencv2/core/core_c.h>
#include <vector>
#include <ext/algorithm>

#define APPROXIMATE_NN
#ifdef APPROXIMATE_NN
  typedef kBinaryMeansThreaded<BriefDesc,uint32_t,LSH<BriefDesc,uint32_t> > kBinaryMeansThreaded_NN;
#else
  typedef kBinaryMeansThreaded<BriefDesc,uint32_t,NearestNeighbor<BriefDesc,uint32_t> > kBinaryMeansThreaded_NN;
#endif

/* Parameters for LSH
 *
 * l=number of hash tables
 * m=number of bits the hash function is looking at
 */
struct LshKBMParams : public Params
{
  LshKBMParams(vector<uint32_t> p_) : Params(p_)
  {};
  LshKBMParams(const LshKBMParams& lshP) : Params(lshP.p)
  {};
  /*
   * doBalanceNullClusters>0  -> balance empty clusaters with overfull clusters
   * doBalanceNullClusters==0 -> do not balance empty clusaters with overfull clusters
   */
  LshKBMParams(uint32_t k, uint32_t l, uint32_t m, uint32_t threadMax, uint32_t maxSamples=0, uint32_t doBalanceNullClusters=1)
  {
    p.push_back(k);
    p.push_back(l);
    p.push_back(m);
    p.push_back(threadMax);
    p.push_back(maxSamples);
    p.push_back(doBalanceNullClusters);
  };
  uint32_t getK() const{return p[0];};
  uint32_t getL() const{return p[1];};
  uint32_t getM() const{return p[2];};
  uint32_t getThreadMax() const{return p[3];};
  uint32_t getMaxSamples() const{return p[4];};
  uint32_t getDoBalanceNullClusters() const{return p[5];};
};

class LshKBM : public Quantizer
{
public:
  LshKBM(LshKBMParams lshKbmP=LshKBMParams(200000,19,31,3,0,1))
    : Quantizer(), mLshKbmP(lshKbmP),
      mpLSH(NULL), mpKBM(NULL), mRnd(time(NULL))
  {
    dims=BriefDesc::BRIEF_K;
    type=CV_8UC1;
    mVer=3;
  };
  ~LshKBM()
  {
    if(mpLSH!=NULL) delete mpLSH;
    if(mpKBM!=NULL) delete mpKBM;
  };

  // return number of visual words -> number of centroids
  virtual unsigned int wordCount() const
  {return mCentroids.size();};

  // create quantizer -> do kBinMeans
  void create(const FeatureDatabase& db,
      std::vector<unsigned int> image_ids);

  // use quantizer to find image?!
  void quantizeForRegistration(QuantizerCallback& cb,
      BriefDesc* query,
      QuantizerContext& ctxt) const;

  void quantizeForRegistration(QuantizerCallback& cb,
      const CvMat *descriptor,
      QuantizerContext& ctxt) const;

  void quantizeForQuery(QuantizerCallback& cb,
      BriefDesc* query,
      QuantizerContext& ctxt) const;

  void quantizeForQuery(QuantizerCallback& cb,
      const CvMat *descriptor,
      QuantizerContext& ctxt,
      bool estimate_P_correct_quant=true) const;

  unsigned char getVersion() const {return mVer;};

  // save the whole thing to a file
  bool save(FILE *f, unsigned char ver) const;
  // load the whole thing from a file
  Quantizer* load(FILE *f);
  // display some information
  void dump(int stats_only = 0) const;

  const vector<BriefDesc*>& getCentroids() const {return mCentroids;}

private:

  LshKBMParams mLshKbmP;
  unsigned char mVer;

  double mJfinal;
  double mDtKBinMeans;

  LSH<BriefDesc,uint32_t>* mpLSH;

  // NN4Cl is the clusterer used for clustering. NNthreaded means the clustering is performed using a threaded Nearest neighbour
  // LSHthreaded means approximate NN is performed using threaded LSH.
  kBinaryMeansThreaded_NN* mpKBM;

  map<BriefDesc*,uint32_t> mVisualWordLUT;
  vector<BriefDesc*> mCentroids; // only used in case the quantizer is loaded from file

  Random mRnd;
  /* prepare LSH tables and visual word LUT from given centroids (mCentroids)
   */
  void prepareFromCentroids(const vector<BriefDesc*>& centroids)
  {
    cout<<"lshKBM::prepareFromCentroids"<<endl;
    assert(centroids.size()>0);
    // generate LUT for visual word lookup
    for(uint32_t i=0; i<centroids.size(); ++i)
    {
      mVisualWordLUT[centroids[i]]=i;
//      cout<<"@"<<i<<": "<<*(mCentroids[i]->pBData)<<endl;
    }
    // prepare LSH for fast approximate NN for queries
    mpLSH = new LSH<BriefDesc,uint32_t>(centroids,LshParams(mLshKbmP.getL(),mLshKbmP.getM()));
    mpLSH->prepare();
  };

  /* prepare centroids (mCentroids) and then LSH and visual word LUT
   * @param mapMPs: points from which to generate the centroids
   */
  void prepare(vector<BriefDesc*>& mapMps)
  {
#ifdef APPROXIMATE_NN
    cout<<"Using approximate NN for kBinaryMeansClustering l="
        <<mLshKbmP.getL()<<" m="<<mLshKbmP.getM()
        <<" nThreads="<<mLshKbmP.getThreadMax()<<endl;
    LshParams clP(mLshKbmP.getL(),mLshKbmP.getM());
#else
    cout<<"Using exhaustive NN for kBinaryMeansClustering"<<endl;
    Params clP;
#endif
    mpKBM = new kBinaryMeansThreaded_NN(mapMps,&clP,mLshKbmP.getThreadMax());
    mJfinal=mpKBM->cluster(mLshKbmP.getK(),mLshKbmP.getDoBalanceNullClusters()>0);
    mCentroids=mpKBM->getCentroids();
    prepareFromCentroids(mCentroids);
  };
};

class LshKBMCallback : public QuantizerCallback
{
  public:
  LshKBMCallback() : mVisualWord(0)
  {};
  ~LshKBMCallback()
  {};
  void quantizedForRegistrationCallback(unsigned int word,
                                           QuantizerContext& ctxt,
                                           bool use_for_norm)
  {
    mVisualWord=word;
  };

  void quantizedForQueryCallback(unsigned int word,
                                    QuantizerContext& ctxt,
                                    double P_correct_quant)
  {
//    cout<<"quantizedForQueryCallback: visual word:"<< word<<endl;
    mVisualWord=word;
  };


  uint32_t mVisualWord;
  private:
};


void loadLshKBM(FeatureDatabase& fdb, LshKBM& lshKBM,
    vector<uint32_t>& imageIds, string pathToLshKBM);



