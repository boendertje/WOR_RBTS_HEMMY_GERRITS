/*
 * RandomParticle.h
 *
 *  Created on: Jun 24, 2020
 *      Author: hemmy
 */

#ifndef RANDOMGENERATOR_HPP_
#define RANDOMGENERATOR_HPP_
#include <random>
class RandomGenerator
{
	public:
		RandomGenerator(double low , double high);
		virtual ~RandomGenerator();
		double giveNextNumber();
	private:
		std::default_random_engine generator;
		std::uniform_real_distribution<double> distribution;
};

#endif /* RANDOMGENERATOR_HPP_ */
