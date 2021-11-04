

#ifndef PARTICLE_HPP
#define PARTICLE_HPP
#include "../Point.hpp"
#include <memory>
namespace Model{ 
	
	class Particle
	{
		public :
			explicit Particle(Point& position,double orientation = 0,double weight = 0.1);
			virtual ~Particle();
			Particle(const Particle& newPar);
			Particle& operator=(Particle& rhs);
			bool operator==(const Particle& rhs) const;
			bool operator>(const Particle& rhs) const;
			bool operator<(const Particle& rhs) const;

			Point& getPositie();
			double getOrientation() const;
			double getWeight() const;
			void setOrientation(double newOrientation);
			void setPositie(const Point& newPos);
			void setWeight(double newWeight);
		private:
			Point position;
			double orientation;
			double weight;
		};
}

#endif
