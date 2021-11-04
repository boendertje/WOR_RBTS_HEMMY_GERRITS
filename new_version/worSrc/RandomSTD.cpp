/*
 * RandomSTD.cpp
 *
 *  Created on: Jun 24, 2020
 *      Author: hemmy
 */

#include "RandomSTD.hpp"
#define SEED_TWO 455677543

RandomSTD::RandomSTD(double mean,double standardDeviation):generator(SEED_TWO),distribution(mean,standardDeviation)
{

}
RandomSTD::~RandomSTD()
{
}
double RandomSTD::giveNextNumber()
{
    return distribution(generator);
}

