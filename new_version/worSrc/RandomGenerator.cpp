/*
 * RandomParticle.cpp
 *
 *  Created on: Jun 24, 2020
 *      Author: hemmy
 */

#include "RandomGenerator.hpp"
#include <iostream>
#define SEED_ONE 100
RandomGenerator::RandomGenerator(double low,double high):generator(SEED_ONE),distribution(low,high)
{
}

RandomGenerator::~RandomGenerator() {
	// TODO Auto-generated destructor stub
}
double RandomGenerator::giveNextNumber()
{
    return distribution(generator);
}

