#include "Controller.hpp"
#include "../RobotWorld.hpp"
#include <memory>
#include "../Goal.hpp"
#include "../MainApplication.hpp"
namespace Model
{
   
    Controller::Controller(AbstractAgent * agent):kalmanController(agent),particleController(agent),filter(NONE)
    {

		std::shared_ptr<AbstractSensor> odo;
        if(Application::MainApplication::isArgGiven("-odom"))
        {
            odo = std::make_shared<OdomSensor>(agent,std::stod(Application::MainApplication::getArg("-odom").value));
        }else
        {
            odo = std::make_shared<OdomSensor>(agent,MEASUREMENTDEVIATION_ODOMETER);
        }
        
  		std::shared_ptr<AbstractSensor> compas;

        if(Application::MainApplication::isArgGiven("-compas"))
        {
            compas = std::make_shared<Compas>(agent,std::stod(Application::MainApplication::getArg("-compas").value));
        }else
        {
            compas = std::make_shared<Compas>(agent,MEASUREMENTDEVIATION_COMPAS);
        }

        std::shared_ptr<AbstractSensor> lidar;

        if(Application::MainApplication::isArgGiven("-lidar"))
        {
            lidar = std::make_shared<Lidar>(agent,std::stod(Application::MainApplication::getArg("-lidar").value));
        }else
        {
            lidar = std::make_shared<Lidar>(agent,MEASUREMENTDEVIATION_LIDAR);
        }
        std::shared_ptr<AbstractActuator> actuator;
        if(Application::MainApplication::isArgGiven("-actuator"))
        {
            actuator = std::make_shared<Actuator>(agent,std::stod(Application::MainApplication::getArg("-actuator").value));
        }else
        {
            actuator = std::make_shared<Actuator>(agent,STD_X_Y);   
        }
        

        agent->attachSensor(odo,true);
		agent->attachSensor(compas,true);
    	agent->attachSensor(lidar,true);
		agent->attachActuator(actuator,true);
    }

    Controller::~Controller()
    {
    }
    
    void Controller::setFilter(FILTERS filter)
    {
        filter = filter;
    }
    void Controller::ini()
    {
        kalmanController.ini();
        particleController.ini();
    }
    void Controller::doNextAction()
    {

        switch (filter)
        {
            case FILTERS::KALMAN:

                    kalmanController.calculateNextAction();
                    kalmanController.update();
            break;
            case FILTERS::BOTHKALMAN:

                    kalmanController.calculateNextAction();
                    kalmanController.update();
                    particleController.setCommand(std::get<0>(kalmanController.getCommand()),std::get<1>(kalmanController.getCommand()));
                    particleController.update();
                  
            break;
            case FILTERS::BOTHPARTICLE:

                    particleController.calculateNextAction();
                    particleController.update();
                    kalmanController.setCommand(std::get<0>(particleController.getCommand()),std::get<1>(particleController.getCommand()));
                    kalmanController.update();
                  
            break;
            case FILTERS::PARTICLE:

                    particleController.calculateNextAction();
                    particleController.update();
                    
            break;
            default:
                break;
        }
        
    }
    Point Controller::getBelieveKalman()
    {
        return kalmanController.believeToPoint();
    }
    Point Controller::getParticleBelieve()
    {
        return particleController.believeToPoint();
    }
    bool Controller::isArrived()
    {
        GoalPtr goal = RobotWorld::getRobotWorld().getGoal("Goal");
        Point goalPosition = goal->getPosition();
        Point kalmanPosition, particlePosition;
           
        kalmanPosition = kalmanController.believeToPoint();
        particlePosition = particleController.believeToPoint();
        double halfSizeX = goal->getSize().x/2;
        double halfSizeY = goal->getSize().y/2;

        return (kalmanPosition.x >= (goalPosition.x - halfSizeX) && kalmanPosition.x <= (goalPosition.x + halfSizeX)
        && kalmanPosition.y >= (goalPosition.y - halfSizeY) && kalmanPosition.y <= (goalPosition.y + halfSizeY)) || (particlePosition.x >= (goalPosition.x - halfSizeX) && particlePosition.x <= (goalPosition.x + halfSizeX)
        && particlePosition.y >= (goalPosition.y - halfSizeY) && particlePosition.y <= (goalPosition.y + halfSizeY)) ;
    }
    
}