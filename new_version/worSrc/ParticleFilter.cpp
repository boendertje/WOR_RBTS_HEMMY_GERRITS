#include "ParticleFilter.hpp"
#include "RandomGenerator.hpp"
namespace Model
{
    ParticleFilter::ParticleFilter(AbstractAgent * agent):agent(agent)
    {

    }

    ParticleFilter::~ParticleFilter()
    {

    }
    void ParticleFilter::actionToParticle(Particle& particle,const std::shared_ptr<MoveActuatorCommand>& command)
    {

            static Actuator actuator(nullptr,agent->getActuator(0)->getStdDev());
            Point temp_particle_position = particle.getPositie();
            std::tuple<Point,double> newPosition;
            newPosition = actuator.drive(temp_particle_position,command->distance,command->angle);
            particle.setPositie(std::get<0>(newPosition));
            particle.setOrientation(std::get<1>(newPosition));
    }
   
    size_t ParticleFilter::findParticleWithTotalWeight(double number,std::vector<Particle>& particles_believe)
    {
        double totalWeight = 0;
        for(size_t i = 0;i < particles_believe.size();++i)
        {
            totalWeight += particles_believe[i].getWeight();
            if(number <= totalWeight)
            {
                return i;
            }
        }
        return particles_believe.size() -1;

    }
  
    void ParticleFilter::filter(std::vector<Particle>& particles_believe_1,const std::shared_ptr<MoveActuatorCommand>& command,const std::vector<std::shared_ptr<AbstractPercept>>& measurements)
    {
        std::vector<Particle> particles_believe = particles_believe_1;
        static Lidar lidar(nullptr,agent->getSensor(SENSORINDEX::LIDAR)->getStdDev());
        static Compas compas(nullptr,agent->getSensor(SENSORINDEX::COMPAS)->getStdDev());

        std::shared_ptr<LidarPercept> measurementParticle;
        double totalWeight = 0;
   
        for(Particle& particle : particles_believe)
        {
            // particle ondergaat actie
            Point position = particle.getPositie();
            actionToParticle(particle,command); 

            // Hoe groot is de kans dat measurement gelijk is aan de measement van de Particle
  
            lidar.setPosition(particle.getPositie());
            measurementParticle = std::static_pointer_cast<LidarPercept>(lidar.getPerceptFor(lidar.getStimulus()));
         double measurementParticleCompas = compas.getMeasurement(position,particle.getPositie());
            double weight = std::static_pointer_cast<LidarPercept>(measurements[1])->determineWeight(measurementParticle);
            weight += (std::static_pointer_cast<DistancePercept>(measurements[0])->determineWeight(measurementParticleCompas)/4);
            totalWeight += weight;
            particle.setWeight(weight);                               
        }
        // Resample op basis van het gewicht.
        RandomGenerator generator(0,totalWeight);
        for(size_t i = 0;i < particles_believe.size();++i)
        {
            particles_believe_1[i] = particles_believe[ParticleFilter::findParticleWithTotalWeight(generator.giveNextNumber(),particles_believe)]; 
        }
    }

}
