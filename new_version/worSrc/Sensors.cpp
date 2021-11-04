#include "Sensors.hpp"
#include "../Shape2DUtils.hpp"
#include "../RobotWorld.hpp"
#include "../Wall.hpp"
#include "../Robot.hpp"
namespace Model
{    
    
    
    OdomSensor::OdomSensor():position(Point(-1,-1))
	{
	}
	/**
	 *
	 */
	OdomSensor::OdomSensor( AbstractAgent* aRobot,double stddev) :
								AbstractSensor( aRobot,stddev),position(Point(-1,-1))
	{
	}
	/**
	 *
	 */
	OdomSensor::~OdomSensor()
	{
	}
	/**
	 *
	 */
	std::shared_ptr<AbstractStimulus> OdomSensor::getStimulus()
	{ 	/*Odom*/
		Point newPosition;
		if(agent){
		Robot* robot = dynamic_cast<Robot*>(agent);
			if(position == Point(-1,-1))
			{
				position = robot->getPosition();
				
			}

			newPosition = robot->getPosition();

		}
		
		static RandomSTD noise(0,stddev);
		double distance = std::sqrt(std::pow(position.x - newPosition.x,2) + std::pow(position.y - newPosition.y,2)) + noise.giveNextNumber();
		position = newPosition;
		
		std::shared_ptr< AbstractStimulus > distanceStimulus(new DistanceStimulus(0,distance));
	
		return distanceStimulus;
	}
	
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept> OdomSensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		DistanceStimulus* distanceStimulus = dynamic_cast< DistanceStimulus* >( anAbstractStimulus.get());
		return std::shared_ptr< AbstractPercept >( new DistancePercept( distanceStimulus->angle,distanceStimulus->distance));
	}
	/**
	 *
	 */
	std::string OdomSensor::asString() const
	{
		return "LaserDistanceSensor";
	}
	/**
	 *
	 */
	std::string OdomSensor::asDebugString() const
	{
		return asString();
	}

	Lidar::Lidar() : AbstractSensor(nullptr,0),position(0,0)
	{
	}
	/**
	 *
	 */
	Lidar::Lidar( AbstractAgent* aRobot, double stddev) : AbstractSensor( aRobot,stddev),position(0,0)
	{
	}
	/**
	 *
	 */
	Lidar::~Lidar()
	{
	}
	/**
	 *
	 */
	std::shared_ptr<AbstractStimulus> Lidar::getStimulus()
	{ 	/*Odom*/
		if(agent)
		{
			Robot* robot = dynamic_cast<Robot*>(agent);
			position = robot->getPosition();
		}

		Matrix<double,180,1> lidar;
		for(size_t row = 0; row < lidar.getRows();++row)
		{
			lidar[row][0] = getSingleMeasurement(position,static_cast<double>(row * 2));
		}
		std::shared_ptr< AbstractStimulus > lidarStimulus(new LidarStimulus(lidar));
		return lidarStimulus;
	}
	void Lidar::setPosition(const Point& aPosition)
	{
		position = aPosition;
	}


	double Lidar::getSingleMeasurement(Point lidarPosition,double angleInDegrees)
	{
	double range  = 1024;
	double distance = range;
    Point tempPoint = getPointFromAngle(lidarPosition,angleInDegrees,range);
   
    	auto walls = Model::RobotWorld::getRobotWorld().getWalls();
    	for(auto wall : walls)
    	{
    		if(Utils::Shape2DUtils::intersect(lidarPosition,tempPoint,wall->getPoint1(),wall->getPoint2()))
    		{
			
    		 Point intersect = Utils::Shape2DUtils::getIntersection(lidarPosition,tempPoint,wall->getPoint1(),wall->getPoint2());
    		 if(distance > getDistanceToPoint(lidarPosition,intersect))
    		 {

    			 distance = getDistanceToPoint(lidarPosition,intersect);
    		 }
			}
    	}
	RandomSTD noise(0,distance/10);
	return distance + noise.giveNextNumber();
}
Point Lidar::getPointFromAngle(Point lidarPosition,double angle , double distance)
{
	return Point(static_cast<int>(std::round(lidarPosition.x+(distance*cos(Utils::MathUtils::toRadians(angle))))),static_cast<int>(std::round(lidarPosition.y+(distance*sin(Utils::MathUtils::toRadians(angle))))));
}
double Lidar::getDistanceToPoint(const Point& start,const Point& end)
{
	return std::sqrt(std::pow(start.x - end.x,2) + std::pow(start.y - end.y,2));
}
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept> Lidar::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		LidarStimulus* distanceStimulus = dynamic_cast< LidarStimulus* >( anAbstractStimulus.get());
		return std::shared_ptr< AbstractPercept >( new LidarPercept( distanceStimulus->distances));
	}
	/**
	 *
	 */
	std::string Lidar::asString() const
	{
		return "Lidar";
	}
	/**
	 *
	 */
	std::string Lidar::asDebugString() const
	{
		return asString();
	}

   
	Compas::Compas( AbstractAgent* aRobot, double stddev) :
								AbstractSensor( aRobot,stddev),angle(0),noise(0,stddev)
	{
	}
	/**
	 *
	 */
	Compas::~Compas()
	{
	}
	/**
	 *
	 */

	std::shared_ptr<AbstractStimulus> Compas::getStimulus()
	{ 	
		Robot* robot = dynamic_cast<Robot*>(agent);
	
		angle = Utils::MathUtils::normaliseDegrees(Utils::MathUtils::toDegrees(Utils::Shape2DUtils::getAngle(robot->getFront())) + noise.giveNextNumber());
		std::shared_ptr< AbstractStimulus > distanceStimulus(new DistanceStimulus(angle,0));
		return distanceStimulus;
	}
	double Compas::getMeasurement(const Point& position,const Point& newPosition)
	{
		
		return Utils::MathUtils::normaliseDegrees(Utils::MathUtils::toDegrees(std::atan2(newPosition.y - position.y,newPosition.x - position.x)) + noise.giveNextNumber());
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept> Compas::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		DistanceStimulus* distanceStimulus = dynamic_cast< DistanceStimulus* >( anAbstractStimulus.get());
		return std::shared_ptr< AbstractPercept >( new DistancePercept( distanceStimulus->angle,distanceStimulus->distance));
	}
	/**
	 *
	 */
	std::string Compas::asString() const
	{
		return "Compas";
	}
	/**
	 *
	 */
	std::string Compas::asDebugString() const
	{
		return asString();
	}
}