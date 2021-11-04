/*
 * Kalmanfilter.h
 *
 *  Created on: Apr 5, 2020
 *      Author: hemmy
 */

#ifndef KALMANFILTER_HPP_
#define KALMANFILTER_HPP_
#include <random>
#include "ignore/Matrix.hpp"
#include "RandomSTD.hpp"
const size_t 	n_robot_believe = 2,
				n_robot_m_believe_data = 2,
				n_robot_a_believe_data = 2;
	typedef Matrix<double,n_robot_believe,n_robot_believe> aMatrix;
	typedef Matrix<double,n_robot_believe,n_robot_a_believe_data> bMatrix;
	typedef Matrix<double,n_robot_m_believe_data,n_robot_believe> cMatrix;
	typedef Matrix<double,n_robot_believe,1> xMatrix;
	typedef Matrix<double,n_robot_m_believe_data,1> zMatrix;
	typedef Matrix<double,n_robot_a_believe_data,1> uMatrix;

namespace Model
{

	class KalmanFilter
	{
		public:


			KalmanFilter();
			KalmanFilter(aMatrix& A,bMatrix& B,cMatrix& C);
			virtual ~KalmanFilter();
			/**
			 * Implementation of the algorithm
			 */
			void filter
			(xMatrix& believe_x_t,Matrix<double,n_robot_believe,n_robot_believe>& spreiding,const zMatrix& zt,const uMatrix& Ut);
			aMatrix& getA();
			bMatrix& getB();
			cMatrix& getC();
			void setR(const aMatrix& r);
			void setA(const aMatrix& a);
			void setB(const bMatrix& b);
			void setC(const cMatrix& c);
			void setQt(const aMatrix& qt);

		private:
			aMatrix A;
			bMatrix B;
			cMatrix C;
			aMatrix R;
			aMatrix Qt;
	};

}
#endif /* KALMANFILTER_HPP_ */
