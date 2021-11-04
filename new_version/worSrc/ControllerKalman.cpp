#include "ControllerKalman.hpp"
#include "Actuator.hpp"
#include "Sensors.hpp"
#include "../MathUtils.hpp"
#include "../Shape2DUtils.hpp"
#include "../AStar.hpp"
#include "../Goal.hpp"
#include "../MainApplication.hpp"
#include "../Logger.hpp"
#include "../Robot.hpp"



namespace Model{


ControllerKalman::ControllerKalman(AbstractAgent * Aagent):AbstractFilterController(Aagent),pathPoint(0)
{
    		
}
void ControllerKalman::ini()
{
		
		U_t[0][0] = 0; /// distance
		U_t[1][0] = 0; /// orientation

		SIGMA[0][0] = agent->getActuator(0)->getStdDev();
		SIGMA[0][1] = 0;
		SIGMA[1][1] = agent->getActuator(0)->getStdDev();
		SIGMA[1][0] = 0;

		aMatrix qt{pow(agent->getSensor(SENSORINDEX::ODOM)->getStdDev(),2),0,0,pow(agent->getSensor(SENSORINDEX::COMPAS)->getStdDev(),2)};
		kalmanfilter.setQt(qt);
		aMatrix r{pow(agent->getActuator(0)->getStdDev(),2),0,0,pow(agent->getActuator(0)->getStdDev(),2)};
		kalmanfilter.setR(r);
		aMatrix A = A.identity();
		cMatrix C = C.identity();
		bMatrix B = B.identity();
		kalmanfilter.setB(B);
		kalmanfilter.setC(C);
		kalmanfilter.setA(A);
		Robot * robot = dynamic_cast<Robot*>(agent);
	
		Point position = robot->getPosition();
		setInitialPosition(position);
		pathPoint = 0;
		astarPath = astar.search(robot->getPosition(),robot->getGoal()->getPosition(),robot->getSize()*1.5);
		update();
}
ControllerKalman::~ControllerKalman()
{

}
void ControllerKalman::update()
{
	Robot * robot = dynamic_cast<Robot*>(agent);



    std::shared_ptr< DistancePercept > odom_reading = std::static_pointer_cast<DistancePercept>(agent->getSensor(SENSORINDEX::ODOM)->getPerceptFor(agent->getSensor(SENSORINDEX::ODOM)->getStimulus()));
    std::shared_ptr< DistancePercept > compas_reading = std::static_pointer_cast<DistancePercept>(agent->getSensor(SENSORINDEX::COMPAS)->getPerceptFor(agent->getSensor(SENSORINDEX::COMPAS)->getStimulus()));
 
	/*maak waarnemingsbelieve op basis van robot believe en sensordata--*/
	xMatrix waarnemingsbelieve;
	waarnemingsbelieve[0][0] = believe[0][0] + (odom_reading->distance ) * cos(Utils::MathUtils::toRadians(compas_reading->angle));
	waarnemingsbelieve[1][0] = believe[1][0] + (odom_reading->distance ) * sin(Utils::MathUtils::toRadians(compas_reading->angle)); 
	zt = kalmanfilter.getC() * waarnemingsbelieve;
	bMatrix b;
	b[0][0] = cos(Utils::MathUtils::toRadians(U_t[1][0]));
	b[1][0] = sin(Utils::MathUtils::toRadians(U_t[1][0]));
	kalmanfilter.setB(b);
	believe_1 = believe;
	kalmanfilter.filter(believe,SIGMA,zt,U_t); 
}

void ControllerKalman::calculateNextAction()
{
	Robot * robot = dynamic_cast<Robot*>(agent);

	
	double maxDistance = 20;
	PathAlgorithm::Vertex vertex(0,0);
	if(pathPoint + 20 < astarPath.size())
	{

	 vertex = astarPath[pathPoint+=static_cast<int>(20)];
		
	}else
	{
		vertex = astarPath[astarPath.size()-1];
	}
	
    double distance = std::sqrt(std::pow(believe[0][0] - vertex.x,2) + std::pow(believe[1][0]- vertex.y,2));
	if(distance > maxDistance)
	 {
	 	distance = maxDistance;
	 }
	
	double dX = vertex.x - believe[0][0];
	double dY = vertex.y - believe[1][0];
	double angle = std::atan2(dY,dX);
	U_t[0][0] = distance;
	U_t[1][0] = Utils::MathUtils::normaliseDegrees(Utils::MathUtils::toDegrees(angle));
	std::shared_ptr<AbstractCommand> command = std::make_shared<MoveActuatorCommand>(U_t[1][0],U_t[0][0]);
	std::shared_ptr<Actuator> motor = std::static_pointer_cast<Actuator>(agent->getActuator(0));
	motor->handleCommand(command);
	believe_1 = believe;
	
}

void ControllerKalman::setInitialPosition(Point iniPos)
{
	believe[0][0] = believe_1[0][0] = iniPos.x;
	believe[1][0] = believe_1[1][0] = iniPos.y;
}

Point ControllerKalman::believeToPoint()
{
		return Point(static_cast<int>(std::round(believe[0][0])),static_cast<int>(std::round(believe[1][0])));
}

void ControllerKalman::setCommand(double speed,double orientationInDegrees)
{
	U_t[0][0] = speed;
	U_t[1][0] = orientationInDegrees;
}
std::tuple<double,double> ControllerKalman::getCommand()
{
	return std::make_tuple(U_t[0][0],U_t[1][0]);
}

}