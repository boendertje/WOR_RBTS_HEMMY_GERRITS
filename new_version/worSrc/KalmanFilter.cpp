/*
 * Kalmanfilter.cpp
 *
 *  Created on: Apr 5, 2020
 *      Author: hemmy
 */

#include "KalmanFilter.hpp"
#include "../MathUtils.hpp"

namespace Model
{

KalmanFilter::KalmanFilter()
{
		A = A.identity();
		C = C.identity();

}
KalmanFilter::KalmanFilter(aMatrix& A,bMatrix& B,cMatrix& C)
:A(A),B(B),C(C)
{
	// TODO Auto-generated constructor stub

}

KalmanFilter::~KalmanFilter()
{
	// TODO Auto-generated destructor stub
}
void KalmanFilter::filter(xMatrix& believe_x_t,Matrix<double,n_robot_believe,n_robot_believe>& spreiding,const zMatrix& zt,const uMatrix& Ut)
{
	// Control update
	xMatrix prodiction_of_mean = A * believe_x_t + B * Ut;
	Matrix<double, n_robot_believe, n_robot_believe> prodiction_of_variance = A * spreiding * A.transpose() + R;
	
	/// measurement
	Matrix<double,n_robot_believe,n_robot_m_believe_data> K = prodiction_of_variance * C.transpose() * (C * prodiction_of_variance * C.transpose() +Qt).inverse();
    believe_x_t = prodiction_of_mean + K * (zt - C * prodiction_of_mean);
    spreiding = (prodiction_of_variance.identity() - K * C) * prodiction_of_variance;
}

aMatrix& KalmanFilter::getA()
{
	return A;
}
bMatrix& KalmanFilter::getB()
{
	return B;
}
cMatrix& KalmanFilter::getC()
{
	return C;
}
void KalmanFilter::setA(const aMatrix& a)
{
	A=a;
}

void KalmanFilter::setR(const aMatrix& r)
{
	R=r;
}
void KalmanFilter::setQt(const aMatrix& qt)
{
	Qt=qt;
}
void KalmanFilter::setB(const bMatrix& b)
{
	B=b;
}
void KalmanFilter::setC(const cMatrix& c)
{
	C=c;
}

}