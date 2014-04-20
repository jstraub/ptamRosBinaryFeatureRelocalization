/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #include <BriefDesc.hpp>
#include <timer.hpp>


int main(int argc, char* argv[])
{
  BriefDescS bd1, bd2;
  std::vector<uint32_t> score1(BriefDesc::N_BITS,10);
  bd1.descriptorFromBriefScore(score1,100);
  for(uint32_t i=0;i<BriefDesc::BRIEF_K;++i)
  {
    cout<<int(bd1.bd[i])<<" ";
    assert(bd1.bd[i]==uint8_t(0));
  } cout<<endl;

  std::vector<uint32_t> score2(BriefDesc::N_BITS,75);

  score2[0]=10;
  for(uint32_t i=1; i<BriefDesc::N_BITS; ++i)
  {
    score2[i-1]=75; score2[i]=10;
    bd2.descriptorFromBriefScore(score2,100);
    uint32_t dist=bd2.dist32_8(bd1); assert(dist==255);
     dist=bd2.dist32_16(bd1); assert(dist==255);
     dist=bd2.distUint8(bd1); assert(dist==255);
     dist=bd2.distIterative(bd1); assert(dist==255);
     dist=bd2.dist16bit(bd1); assert(dist==255);
     cout<<"i="<<i<<" ok; ";
  } cout<<endl;
//  for(uint32_t i=0;i<BriefDesc::BRIEF_K;++i){assert(bd2.bd[i]==uint8_t(255));}

  Timer t0;
  cout<<" ---- timing comparison dist ---- "<<endl;
  uint64_t N=100000000;

  t0.tic();
  for(uint64_t i=0; i<N; ++i){
    uint32_t dist=bd2.dist32_8(bd1);
  }
  t0.toc();
  uint32_t dist=bd2.dist32_8(bd1);cout<<dist<<endl;
  cout<<" 32bit xor 8bit LUT distance:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;

  t0.tic();
  for(uint64_t i=0; i<N; ++i)
    uint32_t dist=bd2.dist32_16(bd1);
  t0.toc();
  dist=bd2.dist32_16(bd1);cout<<dist<<endl;
  cout<<" 32bit xor 16bit LUT distance:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;

  t0.tic();
  for(uint64_t i=0; i<N; ++i)
    uint32_t dist=bd2.distUint8(bd1);
  t0.toc();
  dist=bd2.distUint8(bd1);cout<<dist<<endl;
  cout<<" 8bit xor 8bit LUT distance:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;

  for(uint64_t i=0; i<N; ++i)
    uint32_t dist=bd2.distIterative(bd1);
  t0.toc();
  dist=bd2.distIterative(bd1);cout<<dist<<endl;
  cout<<" iterative distance:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;

  t0.tic();
  for(uint64_t i=0; i<N; ++i)
    bd2.dist16bit(bd1);
  t0.toc();
  dist=bd2.dist16bit(bd1);cout<<dist<<endl;
  cout<<" 16bit LUT distance:\tdt="<<t0.lastDt()/double(N)<<"ms"<<endl;


  return 0;
}
