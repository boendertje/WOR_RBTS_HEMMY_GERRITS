//#include "ParticleFilter.hpp"
//#include "Particle.hpp"

#ifndef CONTROLLERPARTICLE_HPP
#define CONTROLLERPARTICLE_HPP
#include <memory>
#include "AbstractController.hpp"
#include "Particle.hpp"
#include "ParticleFilter.hpp"
#include "../AStar.hpp"

namespace Model{

#define NUMBEROFPARTICLES 5000
#define MEASUREMENTDEVIATION_LIDAR 2

class ControllerParticle : public AbstractFilterController
    {
    private:
        std::vector<Particle> believe_particles;
        ParticleFilter filter;
        int pathPoint;
        PathAlgorithm::Path astarPath;
        PathAlgorithm::AStar astar;
        std::shared_ptr<MoveActuatorCommand> command;
        double determinePlaceCount;
    public:
    ControllerParticle() = delete;
       explicit ControllerParticle(AbstractAgent * agent);
        ~ControllerParticle();
           /**
            * @brief initialize the filter and reset all values
            * 
            **/
        void ini() override;
        void distributeParticles();
        /**
            *@brief calculate and do next Action 
        **/
        void calculateNextAction() override;
        /**
         * @brief find the Particle with the Heighest weight 
         * @return Particle with the heighest weight value
         **/
        Particle& findHighestWeight();
        /**
        * @brief convert Particlebelieve to Point
        * @return Point : believe as a Point value
        **/ 

        Point believeToPoint() override;
        /**
        * @brief  update believe by algorithm
        **/
        void update() override;
        
        std::tuple<double,double> getCommand() override;

        void setCommand(double speed,double orientationInDegrees) override;

    };
}
#endif