#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP
#include "../AbstractActuator.hpp"
#include "RandomSTD.hpp"
#include "../Point.hpp"
namespace Model
{
    class MoveActuatorCommand : public AbstractCommand
    {
       public:
			MoveActuatorCommand():angle(0),distance(0)
		{
        }
		MoveActuatorCommand(double anAngle,
						double aDistance) :
			angle(anAngle),
			distance( aDistance)
		{
		}
		double angle;
		double distance;
	};
    

    class Actuator : public AbstractActuator
    {
        public:
            Actuator();
            Actuator(AbstractAgent* agent,double stddev);
            ~Actuator();
            /**
             * @brief handle the command thats has been giving
             * @param anAbstractCommand : Abstract command that will be performed
             **/
            
            virtual void handleCommand(std::shared_ptr<AbstractCommand> anAbstractCommand) override;  
            std::tuple<Point,double> drive(Point& position,double speed, double orientationInDegrees);
        private:
        protected:
        double stddev;
        double orientation;
    };
}
#endif