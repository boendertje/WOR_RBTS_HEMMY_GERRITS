#ifndef CONTROLLERKALMAN_HPP
#define CONTROLLERKALMAN_HPP
#include "AbstractController.hpp"
#include "KalmanFilter.hpp"
#include "../AStar.hpp"
namespace Model{
    #define MEASUREMENTDEVIATION_ODOMETER 2
    #define MEASUREMENTDEVIATION_COMPAS 2
    #define STD_X_Y 3   
class ControllerKalman : public AbstractFilterController
{
private:
    KalmanFilter kalmanfilter;
	xMatrix believe,believe_1;
	zMatrix zt;
	uMatrix U_t;
    PathAlgorithm::Path astarPath;
    PathAlgorithm::AStar astar;
    int pathPoint;
	Matrix<double,n_robot_believe,n_robot_believe> SIGMA;
public:
    ControllerKalman(/* args */) = delete;
   explicit ControllerKalman(AbstractAgent * Aagent);
    ~ControllerKalman();
    /**
    * @brief  update believe by algorithm
    **/
    void update() override;

   void calculateNextAction() override;
    /**
     * @brief set the Initial position
     * @param iniPos : Point that is representive to the start position
     **/
   void setInitialPosition(Point iniPos);
      /**
     * @brief initialize the filter and reset all values
     * 
     **/
    //@override
    void ini() override;
    /**
     * @brief initialize the filter and reset all values
     * 
     **/
   Point believeToPoint() override;
   

    void setCommand(double speed,double orientationInDegrees) override;
    std::tuple<double,double> getCommand() override;
};
}
#endif