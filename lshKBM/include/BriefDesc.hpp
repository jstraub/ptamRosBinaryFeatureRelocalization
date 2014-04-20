/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef BRIEFDESC_HPP_
#define BRIEFDESC_HPP_

#include <map>
#include <vector>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>

using namespace std;

/* Brief descriptor base class - do not create objects from this type!
 *
 * BriefDesc only works as a base - it does not create handle bd*, which has the
 * actual descriptor.
 */
class BriefDesc
{
public:
  BriefDesc()
  {};

  virtual ~BriefDesc()
  {};

  /* using 8bit lookup table for distance computations
   *
   * Timings for Distance functions:
   *   iterative distance: dt=3.14252e-05ms
   *   8bit LUT distance:  dt=2.38111e-05ms
   *   16bit LUT distance: dt=3.14492e-05ms
   */
  uint32_t dist(BriefDesc* brief2) const
  {
    //return dist32_16(*brief2);
    return dist32_8(*brief2);
//    return distUint8(*brief2);
  }

  uint32_t dist(const BriefDesc& brief2) const
  {
    //return dist32_16(brief2);
    return dist32_8(brief2);
//    return distUint8(brief2);
  }

  inline uint32_t distUint8(const BriefDesc& brief2) const
  {
    assert((bd[0] ^ brief2.bd[0]) < 256);
    uint32_t dist=num8BitLUT[bd[0] ^ brief2.bd[0]];
    for (size_t i=1; i<BRIEF_K; ++i){
      dist+=num8BitLUT[bd[i] ^ brief2.bd[i]];
    }
    return dist;
  };

  inline uint32_t dist32_8(const BriefDesc& brief2) const
  {
//    assert((bd[0] ^ brief2.bd[0]) < 256);
    uint32_t dist=0;
    uint8_t* bd2=brief2.bd;
    for (size_t i=0; i<BRIEF_K; i+=4){
      uint32_t XOR=(*reinterpret_cast<uint32_t*>(bd+i)) ^
                   (*reinterpret_cast<uint32_t*>(bd2+i));
      uint8_t* XORbytes=reinterpret_cast<uint8_t*>(&XOR);
      dist+=num8BitLUT[XORbytes[0]]+num8BitLUT[XORbytes[1]]+num8BitLUT[XORbytes[2]]+num8BitLUT[XORbytes[3]];
    }
    return dist;
  };

// SOMEHOW this is broken!!
//  inline uint32_t dist32_16(const BriefDesc& brief2) const
//  {
////    assert((bd[0] ^ brief2.bd[0]) < 256);
//    uint32_t dist=0;
//    uint8_t* bd2=brief2.bd;
//    for (size_t i=0; i<BRIEF_K; i+=4){
//      uint32_t XOR=(*reinterpret_cast<uint32_t*>(bd+i)) ^
//                   (*reinterpret_cast<uint32_t*>(bd2+i));
//      uint16_t* XOR16=reinterpret_cast<uint16_t*>(&XOR);
//      dist+=num16BitLUT[XOR16[0]]+num16BitLUT[XOR16[1]];
//    }
//    return dist;
//  };

  inline uint32_t distIterative(const BriefDesc& brief2) const
  {
    uint32_t dist=0;
    for (size_t i=0; i<BRIEF_K; ++i)
    {
      uint32_t xOr=bd[i] ^ brief2.bd[i];
      for(int j=0; j<8;++j)
        dist+=((xOr>>j) & 1); // 0 or 1 depending on jth bit in xOr;
    }
    return dist;
  };

  inline uint32_t dist16bit(const BriefDesc& brief2) const
  {
    uint32_t dist=num16BitLUT[((bd[0]<<8)|(bd[1])) ^ ((brief2.bd[0]<<8)|(brief2.bd[1]))];
    for (size_t i=2; i<BRIEF_K; i+=2){
      dist+=num16BitLUT[((bd[i+1]<<8)|(bd[i])) ^ ((brief2.bd[i+1]<<8)|(brief2.bd[i])) ];
    }
    return dist;
  };


  /* count the number of ones in this bit string
   */
  inline uint32_t nOnes() const
  {
    uint32_t n=0;
    for (size_t i=0; i<BRIEF_K; i+=2){
      n+=num16BitLUT[*reinterpret_cast<uint16_t*>(bd+i)];
    }
    return n;
  };

  inline void setZero()
  {
    for(uint32_t i=0; i<N_BYTES; i+=4) *reinterpret_cast<uint32_t*>(bd+i)=0;
  };

  /* increment the uint32_ts at the positions of all ones
   * in the BRIEF descriptor
   */
  template<class T>
  void incrBriefScore(vector<T>& score) const
  {
    assert(score.size()==BRIEF_K*8);
    for (size_t i=0; i<BRIEF_K; ++i)
      for(int j=0; j<8;++j)
        score[i*8+j]+=((bd[i]>>j) & 1); // 0 or 1 depending on jth bit in the BRIEF descriptor;
  };

  /*
   * Fill this descriptor from scored bit positions
   */
  void descriptorFromBriefScore(const vector<uint32_t>& score, uint32_t scoreMax);
  void descriptorFromBriefScoreDiv(const vector<uint32_t>& score, uint32_t scoreMax);

  /* copy data from the other BriefDesc
   */
  virtual void copyDataFrom(BriefDesc* b)
  {
    for(uint32_t i=0; i<BriefDesc::N_BYTES; ++i)
    {
      bd[i]=b->bd[i];
    }
    // do not use memcpy - use save way.
//    memcpy(bd,b->bd,BRIEF_K);
  };

  /* copy data from cvMat data
   */
  virtual void descriptorFromCvMatRow(uint8_t *rowStart){
    memcpy(bd,rowStart,BRIEF_K);
  }

  static const size_t N_BITS=256;
  static const size_t BRIEF_K=N_BITS/8; // assumed to be multiple of 4; can only be 16, 32 and 64 due to openCV
  static const size_t N_BYTES=N_BITS/8;
  static const size_t N_INTS=N_BITS/32;

  static const uint32_t num8BitLUT[256];   // matlab generated lookup table
  static const uint32_t num16BitLUT[65536];
  uint8_t* bd;//[BRIEF_K]; // brief descriptor

private:
  BriefDesc& operator=(const BriefDesc& brief);
};

/* Brief descriptor class which handles bd* itself
 * (S for self)
 */
class BriefDescS : public BriefDesc
{
public:
  BriefDescS() : BriefDesc()
  {
//    cout<<"BriefDescS "<<this<<endl;
    bd=new uint8_t[BRIEF_K];
  };

  BriefDescS(const uint8_t* bd_) : BriefDesc()
  {
    bd=new uint8_t[BRIEF_K];
    memcpy(bd,bd_,BRIEF_K);
  };

  BriefDescS(const BriefDesc* pDesc) : BriefDesc()
  {
//    cout<<"BriefDescS(const BriefDesc* pDesc)"<<endl;
    bd=new uint8_t[BRIEF_K];
    memcpy(bd,pDesc->bd,BRIEF_K);
  };

  virtual ~BriefDescS()
  {
//    cout<<"~BriefDescS "<<this<<endl;
    delete[] bd;
  };
private:
  BriefDescS& operator=(const BriefDescS& brief);
};

/* Brief descriptor class which allows the data to be managed by someone else
 * (M for managed)
 *
 * (important for memory mapped descriptors)
 */
class BriefDescM : public BriefDesc
{
public:
  // constructor in case the briefdata is managed by someone else
  BriefDescM(uint8_t* bd_) : BriefDesc()
  {
    bd=bd_;
  };
  virtual ~BriefDescM()
  {};

private:
  BriefDescM& operator=(const BriefDescM& brief);
};

std::ostream& operator<< (std::ostream &out, const BriefDesc& a);

template<class Desc>
bool loadBriefDesc(const std::string pathToData,
    const std::string fileName,
    std::vector<Desc*>& briefs,
    std::map<Desc*,size_t>& briefFrames)
{
  char buf[fileName.size()+10+pathToData.size()];
  {// load brief data from files
    sprintf(buf,"%s/%s",pathToData.c_str(),fileName.c_str());
    std::ifstream in(buf);
    if(!in.is_open()) return false;
    double w_x, w_y, w_z, i_x,i_y;
    while(in>>w_x)
    {
      BriefDescS* brief=new BriefDescS();
      int32_t frameNr=-1;
//      mp->pBData=brief;
      in>>w_y>>w_z>>frameNr>>i_x>>i_y; // not necessary
      uint32_t d;
      for (size_t j=0; j<BriefDesc::BRIEF_K/4; ++j)
      {
        in>>d;
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
};

// load BRIEF descriptors of keypoints seen in keyFrames
bool loadKeyFrameNrs(const std::string pathToData,
    std::vector<size_t>& keyFrameNrs);

#endif /* BRIEFDESC_HPP_ */
