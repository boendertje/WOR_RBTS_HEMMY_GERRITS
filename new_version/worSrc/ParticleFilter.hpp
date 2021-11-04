#ifndef PARTICLEFILTER_HPP
#define PARTICLEFILTER_HPP
#include "Particle.hpp"
#include <vector>
#include <memory>
#include "Actuator.hpp"
#include "Sensors.hpp"
#include "../Point.hpp"

namespace Model{

    class ParticleFilter
    {
    private:
        /* data */
        AbstractAgent * agent;
    public:
        ParticleFilter(AbstractAgent * agent);
        ~ParticleFilter();
     /**
      * @brief Applies given action on the given Particle.
      * @param Particle : particle represents a point in the world with a weight
      * @param command : MoveCommandPointer, command that has direction and speed 
      **/
       void actionToParticle(Particle& particle,const std::shared_ptr<MoveActuatorCommand>& command); 
        /**
         * @brief find the Particle with the total value of the weight (sum of all the previous weights) 
         *  @param number : number, the totalWeight number that needs to be find
         *  @return size_t : the index of the particle with the given totalWeight
         **/
      static size_t findParticleWithTotalWeight(double number,std::vector<Particle>& particles_believe);
       /**
        * @brief  update believe by algorithm
        **/
      
       void filter(std::vector<Particle>& particles_believe_1,const std::shared_ptr<MoveActuatorCommand>& command
            ,const  std::vector<std::shared_ptr<AbstractPercept>>& measurements);
    };
}
#endif
