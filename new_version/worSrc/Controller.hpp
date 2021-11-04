#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include <memory>

#include "ControllerKalman.hpp"
#include "ControllerParticle.hpp"
#include "../Point.hpp"
#include "../AbstractAgent.hpp"
namespace Model{
     enum FILTERS{
	KALMAN,
	PARTICLE,
	BOTHKALMAN,
    BOTHPARTICLE,
    NONE
	};

class Controller
{
private:
    /* data */

    AbstractAgent * agent;
    ControllerKalman kalmanController;
    ControllerParticle particleController;
    FILTERS filter;
public:
    Controller(/* args */) = delete;
    explicit Controller(AbstractAgent * agent);
    ~Controller();
    void setFilter(FILTERS filter);
    /**
    * @brief calculate and perfrom next action
    **/
    void doNextAction();
    Point getBelieveKalman();
    Point getParticleBelieve();
    /**
     *  @brief calculates if there is a collision based on believe
     *  @return boolean true if collision detected 
     */ 

    /**
     *  @brief calculates if there is a collision based on believe
     *  @return boolean true if agent arrived on goal
     */ 

    bool isArrived();
    /**
    * @brief initialize the different controlles for the algorithms
    */
     void ini();
};

}





#endif