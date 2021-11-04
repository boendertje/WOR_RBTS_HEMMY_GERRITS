/*
 * RandomSTD.h
 *
 *  Created on: Jun 24, 2020
 *      Author: hemmy
 */

#ifndef RANDOMSTD_HPP_
#define RANDOMSTD_HPP_
#include <random>
class RandomSTD
{
	public:
	RandomSTD( ) = delete;
		RandomSTD(double mean,double standardDeviation);
		virtual ~RandomSTD();
		double giveNextNumber();
	private:
		std::default_random_engine generator;
		std::normal_distribution<double> distribution;
};
#endif /* RANDOMSTD_HPP_ */
