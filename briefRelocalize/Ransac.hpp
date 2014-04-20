/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #ifndef RANSAC_HPP_
#define RANSAC_HPP_

#include "MapPoint.h"
#include "BriefData.hpp"
#include "BriefAssociation.hpp"
#include "ATANCamera.h"
#include "Random.hpp"
#include <association.hpp>

#include <classifier.hpp>
#include <MultiThreading.hpp>
#include <results.hpp>

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>

#include <iostream>
#include <math.h>
#include <vector>
#include <stdint.h>
#include <float.h>
#include <algorithm>

using namespace std;
using namespace TooN;

class Complex
{
public:
  double im,re;

  Complex(double re_=0.0, double im_=0.0) : im(im_), re(re_)
  {};
  Complex(const Complex& c2) : im(c2.im), re(c2.re)
  {};

  void toRadial(double& r, double& theta) const
  {
    r=sqrt(re*re+im*im);
    theta=atan2(im,re);
  };
  void fromRadial(const double& r, const double& theta){
    im=r*sin(theta);
    re=r*cos(theta);
  };

  Complex& operator=(const Complex& c2){
    if(this != &c2){
      im=c2.im;
      re=c2.re;
    }
    return *this;
  };

};

ostream& operator<<(ostream& out, const Complex& c);
Complex operator+(const Complex& c1, const Complex& c2);
Complex operator+(const Complex& c1, const double& r2);
Complex operator+(const double& r2, const Complex& c1);
Complex operator-(const Complex& c1, const Complex& c2);
Complex operator/(const Complex& c1, const Complex& c2);
Complex operator/(const double& r1, const Complex& c2);
Complex operator/(const Complex& c,const double& r);
Complex operator*(const Complex& c1, const Complex& c2);
Complex operator*(const double& r, const Complex& c);
Complex operator*(const Complex& c,const double& r);

Complex sqrt(const Complex& c);
Complex cubicrt(const Complex& c);

Matrix<3,3> inverse(Matrix<3,3> A);
Matrix<2,2> inverse(Matrix<2,2> A);

class SimpleCamera
{
public:
  SimpleCamera(const Matrix<2,2,double>& K, const Vector<2,double>& t) :
    mK(K), mt(t), mKinv(inverse(mK)), mtinv(-mKinv*mt)
  {};

  Vector<2> Project(const Vector<2>& camframe){
    return mK*camframe+mt;
  };

  Vector<2> UnProject(const Vector<2>& imframe){
//    cout<<"mKinv:"<<mKinv<<endl;
//    cout<<"mtinv:"<<mtinv<<endl;
    return mKinv*imframe+mtinv;
  };

  string getName() const { return "simpleCam";}

private:
  Matrix<2,2> mK;
  Vector<2> mt;
  Matrix<2,2> mKinv;
  Vector<2> mtinv;
};

/* models a camera calibrated using OpenCV
 * http://opencv.willowgarage.com/documentation/python/camera_calibration_and_3d_reconstruction.html#initintrinsicparams2d
 */
//class OpenCVCam
//{
//public:
//  OpenCVCam(const Matrix<2,2,double>& K, const Vector<2,double>& t, const Vector<5,double>& d) :
//    mK(K), mt(t), mKinv(inverse(mK)), mtinv(-mKinv*mt), md(d)
//  {};
//
//  // camframe -> X/Z and Y/Z
//  Vector<2> Project(const Vector<2>& camframe)
//  {
//    Vector<2> undistCf;
//    double r=camframe[0]*camframe[0]+camframe[1]*camframe[1];
//    double rsq=r*r;
//    undistCf[0]=camframe[0]*(1+md[0]*rsq+md[1]*rsq*rsq+md[4]*rsq*rsq*rsq) + 2*;
//
//    return mK*camframe+mt;
//  };
//
//  Vector<2> UnProject(const Vector<2>& imframe)
//  {
////    cout<<"mKinv:"<<mKinv<<endl;
////    cout<<"mtinv:"<<mtinv<<endl;
//    return mKinv*imframe+mtinv;
//  };
//
//private:
//  Matrix<2,2> mK;
//  Vector<2> mt;
//  Matrix<2,2> mKinv;
//  Vector<2> mtinv;
//
//  Vector<6> md; // distortion params
//};


template<uint32_t N>
class RansacModel
{
public:
  RansacModel(double consenusThreshold) : mConsesusThr(consenusThreshold)
  {};
  virtual ~RansacModel()
  {};

  virtual uint32_t NumDataPerModel(){
    return N;
  };

  double getConsensusThr() const{return mConsesusThr;};

protected:
  const double mConsesusThr; // threshold for calling a point inlier

private:
};


/* Four point pose according to Fischler
 *
 * IMPORTANT: the estimated pose is wTc - the transformation
 *  from world into camera coordinates. (PTAM uses wTc for all computations!)
 */
template<class CM, class Desc>
class FourPoint : public RansacModel<4>
{
public:
  FourPoint(const CM& camModel, double consenusThreshold=CONSENSUS_THR)
  : RansacModel<4>(consenusThreshold), mCamModel(new CM(camModel.getName())) // copy cam model
  {
//    cout<<"mCamModel: "<<mCamModel->getParams()<<endl;
  };
  FourPoint(const FourPoint& fourP)
  : RansacModel<4>(fourP.getConsensusThr()), mCamModel(new CM(fourP.getCamModel()))
    {};
  ~FourPoint()
  {
    delete mCamModel;
  };

  CM& getCamModel() const {return *mCamModel;};

  // pairing[i].mp1 contains the query feature -> we know the image position
  // pairing[i].mp2 contains the matched feature -> we know the 3D position
  bool compute(const vector<Assoc<Desc,uint32_t> >& pairing, TooN::SE3<double>& wTcReturn)
  {
    assert(pairing[0].q!=NULL); assert(pairing[1].q!=NULL);
    assert(pairing[2].q!=NULL); assert(pairing[3].q!=NULL);
    assert(pairing[0].m!=NULL); assert(pairing[1].m!=NULL);
    assert(pairing[2].m!=NULL); assert(pairing[3].m!=NULL);
    assert(pairing.size()==4);
    const uint32_t nPerms=4; // how many permutations are evaluated in order to filter out the true depths
    const uint32_t perm[12]={0, 1, 2,
                       0, 1, 3,
                       0, 2, 3,
                       1, 2, 3}; // TODO: is it sufficient to use less permutations?
    vector<vector<double> > S; // all computed depths
    for(uint32_t i=0; i<nPerms; ++i)
    {
//      cout<<"Computing permutation: "<<perm[i*3+0]<<" "
//          <<perm[i*3+1]<<" "<<perm[i*3+2]<<"."<<endl;
//      cout<<"  "<<pairing[perm[i*3+0]].q->v2p<<endl
//          <<"  "<<pairing[perm[i*3+1]].q->v2p<<endl
//          <<"  "<<pairing[perm[i*3+2]].q->v2p<<endl<<endl;
//      cout<<"  "<<pairing[perm[i*3+0]].m->pt.v3WorldPos<<endl
//          <<"  "<<pairing[perm[i*3+1]].m->pt.v3WorldPos<<endl
//          <<"  "<<pairing[perm[i*3+2]].m->pt.v3WorldPos<<endl<<endl;
      double a=norm(pairing[perm[i*3+1]].m->pt.v3WorldPos-pairing[perm[i*3+2]].m->pt.v3WorldPos);
      double b=norm(pairing[perm[i*3+0]].m->pt.v3WorldPos-pairing[perm[i*3+2]].m->pt.v3WorldPos);
      double c=norm(pairing[perm[i*3+0]].m->pt.v3WorldPos-pairing[perm[i*3+1]].m->pt.v3WorldPos);
//      cout<<"abc: "<<a<<"\t"<<b<<"\t"<<c<<endl;
      Vector<3> j1,j2,j3;
      j1.slice(0,2)=mCamModel->UnProject(pairing[perm[i*3+0]].q->v2p);
      j2.slice(0,2)=mCamModel->UnProject(pairing[perm[i*3+1]].q->v2p);
      j3.slice(0,2)=mCamModel->UnProject(pairing[perm[i*3+2]].q->v2p);
      j1[2]=1.0; j2[2]=1.0; j3[2]=1.0;

//      cout<<"Directions j_i: "<<endl<<j1<<endl<<j2<<endl<<j3<<endl;
      j1=j1/norm(j1);
      j2=j2/norm(j2);
      j3=j3/norm(j3);
//      cout<<"Directions j_i normed: "<<endl<<j1<<endl<<j2<<endl<<j3<<endl;
      double alpha=acos((j2[0]*j3[0]+j2[1]*j3[1]+j2[2]*j3[2]));
      double beta=acos((j1[0]*j3[0]+j1[1]*j3[1]+j1[2]*j3[2]));
      double gamma=acos((j1[0]*j2[0]+j1[1]*j2[1]+j1[2]*j2[2]));
//      cout<<"angles: "<<alpha<<"\t"<<beta<<"\t"<<gamma<<";"<<endl;

      vector<double> coeffs(5,0.0);
      coeffs[0]=4*a*a*c*c*sq(cos(beta))-sq(a*a-b*b+c*c);
      coeffs[1]=-8*a*a*c*c*sq(cos(beta))*cos(gamma)
          -4*c*c*(b*b-c*c+a*a)*cos(alpha)*cos(beta)
          +4*(a*a- b*b)*(a*a-b*b+c*c)*cos(gamma);
      coeffs[2]=4*c*c*(a*a-c*c)*sq(cos(beta))
          +8*c*c*(a*a+b*b)*cos(alpha)*cos(beta)*cos(gamma)
          +4*c*c*(b*b-c*c)*sq(cos(alpha))
          -2*(a*a-b*b-c*c)*(a*a-b*b+c*c)
          -4*sq(a*a-b*b)*sq(cos(gamma));
      coeffs[3]=-4*c*c*(a*a+b*b-c*c)*cos(alpha)*cos(beta)
          -8*b*b*c*c*sq(cos(alpha))*cos(gamma)
          +4*(a*a-b*b-c*c)*(a*a-b*b)*cos(gamma);
      coeffs[4]=4*b*b*c*c*sq(cos(alpha))-sq(a*a-b*b-c*c);

      vector<Complex> roots=solve4thOrder(coeffs);
//      cout<<"roots: ";
//      for(uint32_t j=0; j<roots.size(); ++j){
//        cout<<roots[j]<<"\t";
//      }; cout<<endl;

      const double eps=1e-6;
      for(uint32_t j=0; j<roots.size(); ++j)
        if(fabs(roots[j].im)<eps)
        {
          double u=roots[j].re;
          double v=(-u*u*(a*a-b*b-c*c) -2*(b*b-a*a)*cos(gamma)*u -a*a+b*b-c*c)
            /(2*c*c*(cos(alpha)*u-cos(beta)));
          vector<double> ss(4,0.0);
          ss[perm[i*3+0]]=sqrt((a*a)/(u*u+v*v-2*u*v*cos(alpha)));
          ss[perm[i*3+1]]=u*ss[perm[i*3+0]];
          ss[perm[i*3+2]]=v*ss[perm[i*3+0]];
          S.push_back(ss);
        }//else{ cout<<"Did not accept root: "<<roots[j]<<endl;}
    }

    if(S.size()<2){
//      cout<<"Not enough depths found!"<<endl;
      return false;
    }

//    cout<<"Depths: "<<endl;
//    for(uint32_t i=0; i<S.size(); ++i)
//    {
//      for(uint32_t j=0; j<S[i].size(); ++j)
//        cout<<S[i][j]<<"\t";
//      cout<<endl;
//    }

    vector<Vector<3> > pc;
    vector<Vector<3> > pw;
    {// search for the most similar solutions/depths
      double dMin=FLT_MAX;
      uint32_t iMin=0,jMin=0;
      for(uint32_t i=0; i<S.size(); ++i)
        for(uint32_t j=0; j<i; ++j)
        {
          if(fabs(S[i][0])<1e-10) continue;
          double dist=sqrt((S[i][0]-S[j][0])*(S[i][0]-S[j][0]));
          if(dMin>dist){
            dMin=dist; iMin=i; jMin=j;
          }
        }
      // extract the depths we are going to use
      vector<double> ss(4,0.0);
      for(uint32_t k=0; k<S[iMin].size(); ++k){
        // select depth
        if(fabs(S[iMin][k])<1e-10){
          ss[k]=S[jMin][k];
        }else if(fabs(S[jMin][k])<1e-10){
          ss[k]=S[iMin][k];
        }else{
          ss[k]=(S[jMin][k]+S[iMin][k])*0.5;
        }
        if(ss[k]<0) return false; // negative depths cannot happen (object behind camera?!)

        // compute the position of the point in camera coordinates
        Vector<3> pcc;
        pcc.slice(0,2)=mCamModel->UnProject(pairing[k].q->v2p);
        pcc[2]=1.0;
        pc.push_back(pcc*ss[k]);
        pw.push_back(pairing[k].m->pt.v3WorldPos);
      }
//      cout<<"Final depths: ";
//      for(uint32_t k=0; k<S[iMin].size(); ++k){
//        cout<<ss[k]<<"\t";
//      }; cout<<endl;
//      cout<<"pcs: "<<endl; // seem to be correct
//      for(uint32_t k=0; k<pc.size(); ++k) cout<<pc[k]<<endl;
    }

    { // find the transformation
      // find and subtract means
      uint32_t K=3;
      pcMean[0]=0.0;pcMean[1]=0.0;pcMean[2]=0.0;
      pwMean[0]=0.0;pwMean[1]=0.0;pwMean[2]=0.0;
      for(uint32_t k=0; k<K; ++k) pcMean+=pc[k];
      pcMean/=double(K);
//      cout<<"pcMean="<<pcMean<<endl;
      for(uint32_t k=0; k<K; ++k) pc[k]-=pcMean;
      for(uint32_t k=0; k<K; ++k) pwMean+=pw[k];
      pwMean/=double(K);
//      cout<<"pwMean="<<pwMean<<endl;
      for(uint32_t k=0; k<K; ++k) pw[k]-=pwMean;

//      cout<<"pc:"<<endl;
//      for(uint32_t k=0; k<K; ++k) cout<<pc[k]<<endl;
//      cout<<"pw:"<<endl;
//      for(uint32_t k=0; k<K; ++k) cout<<pw[k]<<endl;

      // do Levenberg Marquardt
      double lambda=1e-4; // lambda from levenberg marquardt
      uint32_t n=0; // number of iterations
      bool converged=false;
      vector<Matrix<3,3> > J(K); // jacobians
      vector<Vector<3> > pcw(K); // camera coordinate points under estimated rotation
      TooN::SO3<double> R; // initialize to zero rotation
      double E=0.0; // Energy
      for(uint32_t k=0; k<K; ++k) E+=norm(pw[k]-R*pc[k]);
      while(!converged && n<20)
      {
        // compute linearization point
        for(uint32_t k=0; k<K; ++k) pcw[k]=R*pc[k];
        // compute jacobians
        for(uint32_t k=0; k<K; ++k){
          J[k][0][0]=0.0; J[k][0][1]=-pcw[k][2]; J[k][0][2]=pcw[k][1];
          J[k][1][0]=pcw[k][2]; J[k][1][1]=0.0; J[k][1][2]=-pcw[k][0];
          J[k][2][0]=-pcw[k][1]; J[k][2][1]=pcw[k][0]; J[k][2][2]=0.0;
        }
        // compute update for parameters
        Vector<3> dx; dx[0]=0.0; dx[1]=0.0; dx[2]=0.0;
        for(uint32_t k=0; k<K; ++k)
        {
          Matrix<3,3> toInv=J[k].T()*J[k];
          toInv[0][0]+=lambda; toInv[1][1]+=lambda; toInv[2][2]+=lambda;
          dx-=inverse(toInv)*J[k].T()*(pw[k]-pcw[k]);
        }
//        cout<<"dx="<<dx<<endl;
        // update
        TooN::SO3<double> Rnew=SO3<double>(dx)*R;
//        cout<<Rnew<<endl<<SO3<double>(dx)<<R<<endl;
        // compute energy under this update
        double Enew=0.0;
        for(uint32_t k=0; k<K; ++k) Enew+=norm(pw[k]-Rnew*pc[k]);
//        cout<<"E="<<E<<" Enew="<<Enew <<" @ iteration "<<n<<endl;
        // decide on step or not step
        if(Enew>E)
          lambda*=10.0;
        else {
          lambda*=0.5;
          R=Rnew;
          if(fabs(E-Enew)<1e-2) converged=true;
          E=Enew;
//          cout<<"new E="<<E<<endl;
        }
        ++n;
      }
      Vector<3> t=pwMean-R*pcMean;
      wTc=TooN::SE3<double>(R,t);
      wTcReturn=wTc;
//      cout<<"after "<<n<<" iterations:"<<endl<<wTc<<endl;
//      cout<<"E="<<E<<endl;
      return true;
    }
  };
  bool compute(const vector<Assoc<Desc,uint32_t> >& pairing)
  {
    SE3<double> wTcReturn;
    return compute(pairing,wTcReturn);
  };

  void consensusSet(const vector<Assoc<Desc,uint32_t> >& pairing,
      vector<Assoc<Desc,uint32_t> >& cs) const
  {
    consensusSet(wTc.inverse(), pairing, cs);
  };
  void consensusSet(const TooN::SE3<double>& cTw, const vector<Assoc<Desc,uint32_t> >& pairing,
      vector<Assoc<Desc,uint32_t> >& cs) const
  {
    if(cs.size()>0) cs.clear();
    // TODO: make sure that compute was called in advance
    for(uint32_t i=0; i<pairing.size(); ++i){
      // project 3D into image
      Vector<3> camCoords=cTw*pairing[i].m->pt.v3WorldPos;
      Vector<2> camFrame=camCoords.slice(0,2)/camCoords[2];
      Vector<2> imFrame=mCamModel->Project(camFrame);

      // check whether projected point is close to the actual measurement
      double dist=norm(imFrame-pairing[i].q->v2p);
//      cout<<dist<<" ";
      if(dist<mConsesusThr)
      { // datapoint lies within the consensus set
        // check whether it is one of the sample points then do not add to consensus set
//        bool isInSamples=false;
//        for(uint32_t j=0; j<sample.size(); ++j)
//          if(data[i]==sample[j])
//          {
//            isInSamples=true;
//            break;
//          }
//        if(!isInSamples) cs.push_back(data[i]);
        cs.push_back(pairing[i]);
      }else{
//        cout<<" --- "<<endl;
//        cout<<"W 3D pos:\t" <<pairing[i].mp2->v3WorldPos<<endl;
//        cout<<"cTw: "<<endl<<cTw;
//        cout<<"C 3D pos:\t" <<camCoords<<endl;
//        cout<<"CamFrame:\t" <<camFrame<<endl;
//        cout<<"imFrame:\t" <<imFrame<<endl;
//        cout<<"imFrame real:\t" <<pairing[i].mp1->pBData->v2p<<endl;
//        cout<<"Dist:\t"<<dist<<endl;
      }
    }
//    cout<<endl;
  };

  uint32_t consensusSize(const TooN::SE3<double>& cTw,
      const vector<Assoc<Desc,uint32_t> >& pairing)
  {
    vector<Assoc<Desc,uint32_t> > cs;
    consensusSet(cTw, pairing, cs);
    return cs.size();
  };

  const SE3<double>& refineModel(const vector<MapPoint*>& cs)
  {
    uint32_t K=cs.size();
    TooN::SO3<double> R=wTc.get_rotation();
    Vector<3> t=wTc.get_translation();

    cout<<"Refinement: pwMean:" << pwMean<<endl;
    pwMean[0]=0.0;pwMean[1]=0.0;pwMean[2]=0.0;
    for(uint32_t k=0; k<K; ++k) pwMean+=cs[k]->v3WorldPos;
    cout<<"K="<<K<<endl;
    cout<<"Refinement: pwMean:" << pwMean<<endl;
    pwMean/=double(K);
    cout<<"Refinement: pwMean:" << pwMean<<endl;

    t=pwMean-R*pcMean;
    cout<<"Refinement: wTc:"<<endl<<wTc;
    wTc=TooN::SE3<double>(R,t);
    cout<<"Refinement: wTc:"<<endl<<wTc;

    return wTc;
  };

  static const double CONSENSUS_THR=9.0; // pixel

private:

  CM* mCamModel;

  TooN::SE3<double> wTc;
  Vector<3> pcMean;
  Vector<3> pwMean;

  double sq(const double a) const
  {
    return a*a;
  };
  // coeffs[4] is highest coeff
  vector<Complex> solve4thOrder(const vector<double>& coeffs) const
  {
    assert(coeffs.size()==5);
    // solve 4th order polynomial
    double a3 = coeffs[3]/coeffs[4];
    double a2 = coeffs[2]/coeffs[4];
    double a1 = coeffs[1]/coeffs[4];
    double a0 = coeffs[0]/coeffs[4];
//      cout<<"Polynomial: 1;"<<a3<<"\t"<<a2<<"\t"<<a1<<"\t"<<a0<<"\t"<<endl;
    double T1 = -a3/4.0;
    double T2 = a2*a2 - 3.0*a3*a1 + 12.0*a0;
    double T3 = (2.0*a2*a2*a2 - 9.0*a3*a2*a1 + 27.0*a1*a1 + 27.0*a3*a3*a0 - 72.0*a2*a0)*0.5;
    double T4 = (-a3*a3*a3 + 4.0*a3*a2 - 8.0*a1)/32.0;
    double T5 = (3.0*a3*a3 - 8.0*a2)/48.0;
//      cout<<"Ts: "<<T1<<"\t"<<T2<<"\t"<<T3<<"\t"<<T4<<"\t"<<T5<<endl;
    Complex R1 = sqrt(Complex(T3*T3 - T2*T2*T2));
    Complex R2 = cubicrt(T3 + R1);
    Complex R3 = (T2/R2 + R2)/12.0;
    Complex R4 = sqrt(T5 + R3);
    Complex R5 = 2.0*T5 - R3;
    Complex R6 = T4/R4;
//      cout<<"Rs: "<<R1<<"\t"<<R2<<"\t"<<R3<<"\t"<<R4<<"\t"<<R5<<"\t"<<R6<<endl;

    vector<Complex> roots;
    roots.push_back(Complex(T1 - R4 - sqrt(R5 - R6)));
    roots.push_back(Complex(T1 - R4 + sqrt(R5 - R6)));
    roots.push_back(Complex(T1 + R4 - sqrt(R5 + R6)));
    roots.push_back(Complex(T1 + R4 + sqrt(R5 + R6)));
    return roots;
  };

};


struct RansacParams : public Params
{
  RansacParams(vector<uint32_t> p_) : Params(p_)
  {};
  RansacParams(const Params& clP) : Params(clP)
  {};
  RansacParams(const RansacParams& lshP) : Params(lshP.p)
  {};
  RansacParams(uint32_t minInliers, uint32_t maxIterations)
  {
    p.push_back(minInliers);
    p.push_back(maxIterations);
  };

  uint32_t getMinInliers() const    {return p[0];};
  uint32_t getMaxIterations() const {return p[1];};

};

/* RANSAC algorithm
 *
 * IMPORTANT: the estimated pose is wTc if the model is FourPoint
 *  - the transformation from world into camera coordinates.
 *  (PTAM uses wTc for all computations!)
 */
template<class Model, class Desc>
class Ransac
{
public:
  Ransac(Random& rnd, const Model& model, const RansacParams& ransacParams)
  : mRnd(rnd), mModel(model), mRansacParams(ransacParams)
  {};
  Ransac(const Ransac& ransac)
  : mRnd(ransac.mRnd), mModel(ransac.mModel, ransac.mModel.mConsensusThr),
    mRansacParams(ransac.mRansacParams)
  {};
  ~Ransac()
  {};

  //PTAM uses wTc for all computations!
  SE3<double> find(const vector<Assoc<Desc,uint32_t> >& pairing)
  {
    return findAmong(pairing,pairing.size());
  };
  //PTAM uses wTc for all computations!
  // deprecated - use Prosac
//  SE3<double> findAmongBest(vector<Assoc<Desc,uint32_t> >& pairing, uint32_t nBest=0)
//  {
//    assert(false);
//    sort(pairing.begin(),pairing.end());
//    return findAmong(pairing,nBest);
//  };
  //PTAM uses wTc for all computations!
  SE3<double> findAmong(const vector<Assoc<Desc,uint32_t> >& pairing, uint32_t nAmong=99999)
  {
    if(pairing.size() < mModel.NumDataPerModel()) return SE3<double>();
    TooN::SE3<double> wTc;
    TooN::SE3<double> LatestwTc;
    int32_t maxInlier=-1;
    uint32_t N=pairing.size();
    uint32_t i=0;

    if(nAmong>0) nAmong=min(nAmong,N);
//    else
//      nAmong=N>1000?N*0.25:250;

    while(i<mRansacParams.getMaxIterations())
    {
      vector<Assoc<Desc,uint32_t> > sample;
      mRnd.resetRepetitions();
      while(sample.size()<mModel.NumDataPerModel())
      {
        uint32_t id=mRnd.drawWithoutRepetition(nAmong);
        bool equalQueryFeatures=false;
        for(uint32_t i=0; i<sample.size(); ++i)
          if(sample[i].q==pairing[id].q)
          {
            equalQueryFeatures=true;
            break;
          }
        // duplicates are possible since lsh might match a query feature to multiple features with equal dist
        if(!equalQueryFeatures){ sample.push_back(pairing[id]);}else{
          cout<<"Ransac: Avoided duplicated query feature in sample for model generation!"<<endl;
        }
      };

      // compute model from the sampled datapoints
//      cout<<"Ransac::find: compute model! iteration "<<i<<endl;
      ++i;
      if(!mModel.compute(sample,LatestwTc)) continue;
      uint32_t nInlier=mModel.consensusSize(LatestwTc.inverse(),pairing);;
//      cout<<"Ransac::find: nInlier="<<nInlier<<endl;

      // model is good enough -> remember
//        mModel.refineModel(cs);
//        cs.clear();
//        mModel.consensusSet(data,sample,cs);
//        uint32_t nInlier=cs.size();
//        cout<<"nInlier="<<nInlier<<endl;
      if(int32_t(nInlier)>maxInlier){
        maxInlier=nInlier;
        wTc=LatestwTc;
//        cout<<"nInlier="<<nInlier<<endl;
//        cout<<"Ransac::find: LatestwTc:"<<endl<<LatestwTc;
      }
    }
//    if(! maxInlier>=mMinInliers){
//      return SE3<double>();
//    }else{
      cout<<"Inliers of best model: "<<maxInlier<<endl
          <<"Model: "<<endl<<wTc;
      mInlierCount=maxInlier;
      return wTc;
//    }
  }

  uint32_t getInlierCount() const { return mInlierCount;};

  Random& mRnd;
  Model mModel;

  const RansacParams mRansacParams;

private:
  uint32_t mInlierCount;
};

template<class Model, class Desc>
class Prosac
{
public:
  Prosac(Random& rnd, const Model& model, const RansacParams& ransacParams)
  : mRnd(rnd), mModel(model), mRansacParams(ransacParams)
  {};
  Prosac(const Prosac& prosac)
  : mRnd(prosac.mRnd), mModel(prosac.mModel, prosac.mModel.mConsensusThr),
    mRansacParams(prosac.mRansacParams)
  {};
  ~Prosac()
  {};

  //PTAM uses wTc for all computations!
  SE3<double> find(vector<Assoc<Desc,uint32_t> >& pairing)
    {
   return findAmongBest(pairing,0);
    };
  //PTAM uses wTc for all computations!
  // nBest is ignored - just for Ransac compatibility
  SE3<double> findAmongBest(vector<Assoc<Desc,uint32_t> >& pairing, uint32_t nBest=0)
  {
    sort(pairing.begin(),pairing.end());

    if(pairing.size() < mModel.NumDataPerModel()) return SE3<double>();
    TooN::SE3<double> wTc;
    TooN::SE3<double> LatestwTc;
    int32_t maxInlier=-1;
    uint32_t Tmax=mRansacParams.getMaxIterations();
    uint32_t m=mModel.NumDataPerModel();
    uint32_t TN=nIterationsRansac(P_GOOD_SAMPLE, OUTLIERS_PROPORTION, m,INT_MAX); // iterations necessary according to RANSAC formula
    uint32_t N=pairing.size();
    uint32_t n=m;
    uint32_t TnPrime=1, t=0;
    uint32_t InlierNStar=0, kNStar=TN;
    double Tn=TN;
    for(uint32_t i=0; i<m; i++) {
        Tn *= (double)(n-i)/(N-i);
    }
    cout<<"   #pairings="<<pairing.size()<<" Tmax="<<Tmax<<endl;
    while(t<=kNStar && t<TN && t<Tmax)
    {
      if((t>TnPrime)&&(n<N))
      { // adjust sampling size n
        double TnNext=(Tn*(n+1))/(n+1-m);
        n++;
        TnPrime=TnPrime + uint32_t(ceil(TnNext-Tn));
        Tn=TnNext;
      }

      vector<Assoc<Desc,uint32_t> > sample; sample.reserve(m);
      if(t>TnPrime)
      { // standard RANSAC - draw from complete set
        mRnd.resetRepetitions();
        while(sample.size()<m){
          uint32_t id=mRnd.drawWithoutRepetition(N);
          bool equalQueryFeatures=false;
          for(uint32_t i=0; i<sample.size(); ++i)
            if(sample[i].q==pairing[id].q)
            {
              equalQueryFeatures=true;
              break;
            }
          // duplicates are possible since lsh might match a query feature to multiple features with equal dist
          if(!equalQueryFeatures){
            assert(pairing[id].m!=NULL);
            sample.push_back(pairing[id]);
          }else{
            cout<<"Prosac: Avoided duplicated query feature in sample for model generation!"<<endl;
          }
        };
//        cout<<"@"<<t<<": draw "<<m<<" out of "<<n<<endl;
      }else{
        // prosac - draw from nth correspondence and the set U_{n-1}
        sample.push_back(pairing[n]);
        mRnd.resetRepetitions();
        uint32_t j=1;
        while(sample.size()<m){
          uint32_t id=mRnd.drawWithoutRepetition(n-1);
          bool equalQueryFeatures=false;
          for(uint32_t i=0; i<sample.size(); ++i)
            if(sample[i].q==pairing[id].q)
            {
              equalQueryFeatures=true;
              break;
            }
          // duplicates are possible since lsh might match a query feature to multiple features with equal dist
          if(!equalQueryFeatures){
            assert(pairing[id].m!=NULL);
            sample.push_back(pairing[id]);
          }else{
            cout<<"Prosac: Avoided duplicated query feature in sample for model generation!"<<endl;
          }
          // increase n in order to be able to find a model
          if(j>=n-1) ++n;
          ++j;
        };
//        cout<<"@"<<t<<": draw "<<m<<" out of "<<n<<endl;
      }

      // compute model from the sampled datapoints
//      cout<<"Ransac::find: compute model! iteration "<<i<<endl;
      ++t;
      if(!mModel.compute(sample,LatestwTc)) continue;
      uint32_t nInlier=mModel.consensusSize(LatestwTc.inverse(),pairing);;
//      cout<<"Ransac::find: nInlier="<<nInlier<<endl;

      // model is good enough -> remember
//        mModel.refineModel(cs);
//        cs.clear();
//        mModel.consensusSet(data,sample,cs);
//        uint32_t nInlier=cs.size();
//        cout<<"nInlier="<<nInlier<<endl;

      if(int32_t(nInlier)>maxInlier){
        maxInlier=nInlier;
        wTc=LatestwTc;
        if(maxInlier>int32_t(InlierNStar))
        { // estimate new upper bound on the number of iterations necessary to find a good model
          // (maximality criterion from PROSAC paper)
          InlierNStar=maxInlier;
          kNStar=nIterationsRansac(1.0-ETA0,1.0-double(maxInlier)/double(N), m,Tmax);
        }
//        cout<<"nInlier="<<nInlier<<endl;
//        cout<<"Ransac::find: LatestwTc:"<<endl<<LatestwTc;
      }
    }
//    if(! maxInlier>=mMinInliers){
//      return SE3<double>();
//    }else{
      cout<<"Inliers of best model: "<<maxInlier<<" #trials="<<t<<endl
          <<"Model: "<<endl<<wTc;
      mInlierCount=maxInlier;
      return wTc;
//    }
  }

  uint32_t getInlierCount() const { return mInlierCount;};

  Random& mRnd;
  Model mModel;

  const RansacParams mRansacParams;

private:
  uint32_t mInlierCount;

//  maximality – the probability that a solution with more
// than In∗ inliers in Un∗ exists and was not found after
// k samples is smaller than η0 (typically set to 5%).
  static const double ETA0=0.01;
  static const double P_GOOD_SAMPLE=0.95;
  static const double OUTLIERS_PROPORTION=0.95;

  uint32_t nIterationsRansac(double pSampleAllInliers,
      double pOutliers, uint32_t nSamples, uint32_t Nmax)
  {
    int32_t N=int32_t(ceil(log(1.0-pSampleAllInliers)/log(1.0-pow(1.0-pOutliers,double(nSamples)))));
    if (N>=0 && uint32_t(N)<Nmax)
      return uint32_t(N);
    else
      return Nmax;
  };
};

template <class Mod, class Desc>
class RansacReLoc : public SingleThread<LocResult>
{
public:
  RansacReLoc(Mod* model, const RansacParams& ransacParams,
      Classifier<Desc,uint32_t>* classifier, const vector<Desc*>& queryMPs, uint32_t kNN=1)
  :   SingleThread<LocResult>(),
      mQueryMPs(queryMPs),
      mpClassifier(classifier),
      mRnd(time(NULL)),
      mpModel(model),
      mRansac(mRnd,*mpModel,ransacParams), mKNN(kNN)
  {
      mRes.retrieved.resize(queryMPs.size(),0);
  };
  ~RansacReLoc()
  {
    delete mpModel;
    delete mpClassifier;
  };

protected:

  void doWork_impl()
  {// work in here
    vector<Assoc<Desc,uint32_t> > pairing;
    vector<uint32_t> retrieved;
    cout<<"Prepairing Classifier!"<<endl;
    float tPrep=mpClassifier->prepare();
    cout<<"Classifying!"<<endl;
    float tPair=0.0;
    if(mKNN>1)
      tPair=mpClassifier->pair(mQueryMPs,pairing,retrieved,mKNN);
    else
      tPair=mpClassifier->pair(mQueryMPs,pairing,retrieved);

    cout<<"Starting RANSAC!"<<endl;
    TooN::SE3<double> wTc=mRansac.findAmongBest(pairing,pairing.size()/4);

    // save results and statistics
    boost::mutex::scoped_lock l(mResMutex); // lock before assigning values

    mRes.p3d.reserve(pairing.size());
    for(uint32_t i=0; i<pairing.size(); ++i){
      mRes.p3d.push_back(pairing[i].m->pt.v3WorldPos);
    }

    mRes.wTc=wTc;
    mRes.inliers=mRansac.getInlierCount();
    mRes.querySize=pairing.size();//mQueryMPs.size();
    mRes.tPrep=tPrep;
    mRes.tPair=tPair;
    mRes.retrieved=retrieved;
    evalPairing(pairing, mRes.avgDist, mRes.paired);
    //compairAgainstNN(pairingLSH,mPairingNN, mCRes.tp);
  };

private:

  vector<Desc*> mQueryMPs;
  Classifier<Desc,uint32_t>* mpClassifier;
  Random mRnd;
  Mod* mpModel;
  Ransac<Mod,Desc> mRansac;
  uint32_t mKNN;
};


//template<class Model>
//class PremtiveRansac
//{
//public:
//  PremtiveRansac(Random& rnd, const Model& model, uint32_t minInliers=10,
//      uint32_t maxIterations=300)
//  : mRnd(rnd), mModel(model), mMinInliers(minInliers),
//    mMaxIterations(maxIterations)
//  {};
//  PremtiveRansac(const PremtiveRansac& ransac)
//  : mRnd(ransac.mRnd), mModel(ransac.mModel, ransac.mModel.mConsensusThr),
//    mMinInliers(ransac.mMinInliers), mMaxIterations(ransac.mMaxIterations)
//  {};
//  ~Ransac()
//  {};
//
//  TooN::SE3<double> find(const vector<BriefAssociation>& pairing)
//  {
//    return findAmong(pairing,pairing.size());
//  };
//
//  TooN::SE3<double> findAmongBest(vector<BriefAssociation>& pairing, uint32_t nBest=100)
//  {
//    sort(pairing.begin(),pairing.end());
//    return findAmong(pairing,nBest);
//  };
//
//  TooN::SE3<double> findAmong(const vector<BriefAssociation>& pairing, uint32_t nAmong=100)
//  {
//    assert(pairing.size() < mModel.NumDataPerModel());
//    vector<Model> models;
//    for(uint32_t i=0;i<mMaxIterations;++i)
//    {
//      Model model;
//      vector<BriefAssociation> sample;
//      for(uint32_t j=0; j<mModel.NumDataPerModel(); ++j){
//        uint32_t id=mRnd.drawWithoutRepetition(nAmong,j==0);
//        sample.push_back(pairing[id]);
//      };
//      // compute model from the sampled datapoints
////      cout<<"Ransac::find: compute model! iteration "<<i<<endl;
//      if(!model.compute(sample)) continue;
//      models.push_back(model);
//    }
//
//
//
//
//    TooN::SE3<double> wTc;
//    TooN::SE3<double> LatestwTc;
//    int32_t maxInlier=-1;
//    uint32_t N=pairing.size();
//    uint32_t i=0;
//    nAmong=min(nAmong,N);
//    while(i<mMaxIterations)
//    {
//      vector<BriefAssociation> sample;
//      for(uint32_t j=0; j<mModel.NumDataPerModel(); ++j){
//        uint32_t id=mRnd.drawWithoutRepetition(nAmong,j==0);
//        sample.push_back(pairing[id]);
//      };
//
//      // compute model from the sampled datapoints
////      cout<<"Ransac::find: compute model! iteration "<<i<<endl;
//      if(!mModel.compute(sample,LatestwTc)) continue;
//      uint32_t nInlier=mModel.consensusSize(LatestwTc,pairing);;
////      cout<<"Ransac::find: nInlier="<<nInlier<<endl;
//
//      // model is good enough -> remember
////        mModel.refineModel(cs);
////        cs.clear();
////        mModel.consensusSet(data,sample,cs);
////        uint32_t nInlier=cs.size();
////        cout<<"nInlier="<<nInlier<<endl;
//      if(int32_t(nInlier)>maxInlier){
//        maxInlier=nInlier;
//        wTc=LatestwTc;
////        cout<<"nInlier="<<nInlier<<endl;
////        cout<<"Ransac::find: LatestwTc:"<<endl<<LatestwTc;
//      }
//      ++i;
//    }
////    if(! maxInlier>=mMinInliers){
////      return SE3<double>();
////    }else{
//      cout<<"Inliers of best model: "<<maxInlier<<endl
//          <<"Model: "<<endl<<wTc;
//      mInlierCount=maxInlier;
//      return wTc;
////    }
//  }
//
//  uint32_t getInlierCount() const { return mInlierCount;};
//
//  Random& mRnd;
//  Model mModel;
//
//  const uint32_t mMinInliers;
//  const uint32_t mMaxIterations;
//
//private:
//  uint32_t mInlierCount;
//};


#endif /* RANSAC_HPP_ */
