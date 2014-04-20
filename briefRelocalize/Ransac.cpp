/* Copyright (c) 2013, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the GPLv3 license. See the license file LICENSE.
 * 
 * If this code is used, the following should be cited:  
 * [1] Fast Relocalization For Visual Odometry Using Binary Features (J.
 * Straub, S. Hilsenbeck, G. Schroth, R. Huitl, A. Moeller, E. Steinbach), In 
 * IEEE International Conference on Image Processing (ICIP), 2013 
 */
 #include "Ransac.hpp"

ostream& operator<<(ostream& out, const Complex& c){
  if(c.im<0.0)
    out<<c.re<<c.im<<"i";
  else
    out<<c.re<<"+"<<c.im<<"i";
  return out;
};

Complex operator+(const Complex& c1, const Complex& c2){
  return Complex(c1.re+c2.re,c1.im+c2.im);
};
Complex operator+(const Complex& c1, const double& r2){
  return c1+Complex(r2,0.0);
};
Complex operator+(const double& r2, const Complex& c1){
  return c1+Complex(r2,0.0);
};
Complex operator-(const Complex& c1, const Complex& c2){
  return c1+Complex(-c2.re,-c2.im);
};
Complex operator/(const Complex& c1, const Complex& c2){
  Complex erg;
  double r1,theta1,r2,theta2;
  c1.toRadial(r1,theta1);
  c2.toRadial(r2,theta2);
  erg.fromRadial(r1/r2,theta1-theta2);
  return erg;
};
Complex operator/(const double& r1, const Complex& c2){
  Complex erg;
  double r2,theta2;
  c2.toRadial(r2,theta2);
  erg.fromRadial(r1/r2,-theta2);
  return erg;
};
Complex operator/(const Complex& c,const double& r){
  return Complex(c.re/r,c.im/r);
};
Complex operator*(const Complex& c1, const Complex& c2){
  Complex erg;
  double r1,theta1,r2,theta2;
  c1.toRadial(r1,theta1);
  c2.toRadial(r2,theta2);
  erg.fromRadial(r1*r2,theta1+theta2);
  return erg;
};
Complex operator*(const double& r, const Complex& c){
  return Complex(c.re*r,c.im*r);
};
Complex operator*(const Complex& c,const double& r){
  return Complex(c.re*r,c.im*r);
};

Complex sqrt(const Complex& c) {
  Complex erg;
  double r, theta;
  c.toRadial(r,theta);
  erg.fromRadial(sqrt(r),theta/2.0);
  return erg;
};
Complex cubicrt(const Complex& c) {
  Complex erg;
  double r, theta;
  c.toRadial(r,theta);
  erg.fromRadial(pow(r,1.0/3.0),theta/3.0);
  return erg;
};

Matrix<3,3> inverse(Matrix<3,3> A)
{
  Matrix<3,3> Ainv;

  double detA=A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*A[2][1]
      -A[0][2]*A[1][1]*A[2][0]-A[0][0]*A[1][2]*A[2][1]-A[0][1]*A[1][0]*A[2][2];

  Ainv[0][0]=(A[1][1] * A[2][2] - A[1][2] * A[2][1])/detA;
  Ainv[0][1]=(A[0][2] * A[2][1] - A[0][1] * A[2][2])/detA;
  Ainv[0][2]=(A[0][1] * A[1][2] - A[0][2] * A[1][1])/detA;
  Ainv[1][0]=(A[1][2] * A[2][0] - A[1][0] * A[2][2])/detA;
  Ainv[1][1]=(A[0][0] * A[2][2] - A[0][2] * A[2][0])/detA;
  Ainv[1][2]=(A[0][2] * A[1][0] - A[0][0] * A[1][2])/detA;
  Ainv[2][0]=(A[1][0] * A[2][1] - A[1][1] * A[2][0])/detA;
  Ainv[2][1]=(A[0][1] * A[2][0] - A[0][0] * A[2][1])/detA;
  Ainv[2][2]=(A[0][0] * A[1][1] - A[0][1] * A[1][0])/detA;

  return Ainv;
};
Matrix<2,2> inverse(Matrix<2,2> A){
  Matrix<2,2> Ainv;
  double detA=A[0][0]*A[1][1]-A[0][1]*A[1][0];
  Ainv[0][0]=A[1][1];  Ainv[0][1]=-A[0][1];
  Ainv[1][0]=-A[1][0]; Ainv[1][1]=A[0][0];
  return Ainv*(1/detA);
};
