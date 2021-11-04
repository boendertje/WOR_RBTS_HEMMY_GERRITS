#ifndef ABSTRACTCONTROLLER_HPP
#define ABSTRACTCONTROLLER_HPP
#include "../AbstractAgent.hpp"
#include "ignore/Matrix.hpp"
#include "../Point.hpp"
namespace Model{


class AbstractFilterController
{
protected:
    AbstractAgent * agent;
public:
    AbstractFilterController();
    AbstractFilterController(AbstractAgent * agent);
   virtual ~AbstractFilterController();
   /**
   * @brief calculate and do next Action 
   **/

   virtual void calculateNextAction() = 0;
    /**
    * @brief  update believe by algorithm
    **/

   virtual void update() = 0;
    /**
     * @brief initialize the filter and reset all values
     * 
     **/

   virtual void ini() = 0;
   /**
    * @brief convert believe to Point
    * @return Point : believe as a Point value
    **/ 
   virtual Point believeToPoint() = 0;

   /*
   * @Brief set a command given the different parameters
   * @param speed : speed
   * @param orientationInDegrees : orientation in degrees
   */

   virtual void setCommand(double speed,double orientationInDegrees) = 0;
   virtual std::tuple<double,double> getCommand() = 0;
};
}
#endif
