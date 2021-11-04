#include "Particle.hpp"

namespace Model{

Particle::Particle(Point& position,double orientation,double weight):position(position),orientation(orientation),weight(weight)
{
}
Particle::~Particle()
{

}
Particle::Particle(const Particle& newPar)
{

	if(this != &newPar)
	{
			position = newPar.position;
			orientation = newPar.orientation;
			weight = newPar.weight;
	}

}
Point& Particle::getPositie()
{
	return position;
}
double Particle::getWeight() const
{
	return weight;
}
void Particle::setPositie(const Point& newPos)
{
	position = newPos;
}
double Particle::getOrientation() const
{
	return orientation;
}
void Particle::setOrientation(double newOrientation)
{
	orientation = newOrientation;
}
void Particle::setWeight(double newWeight)
{
	weight = newWeight;
}

Particle& Particle::operator=(Particle& rhs)
{
	if (this != &rhs)
	{
			position = rhs.position;
			weight = rhs.weight;
			orientation = rhs.orientation;
	}
	return *this;
}
bool Particle::operator==(const Particle& rhs) const
{
	return position == rhs.position && rhs.weight == weight && orientation == rhs.orientation;
}
bool Particle::operator>(const Particle& rhs) const
{
	return weight > rhs.weight;
}
bool Particle::operator<(const Particle& rhs) const
{
	return weight < rhs.weight;
}

}