#ifndef SENSOREN_HPP
#define SENSOREN_HPP
#include "../AbstractSensor.hpp"
#include "../AbstractAgent.hpp"
#include "../Point.hpp"
#include "RandomSTD.hpp"


namespace Model
{
	enum SENSORINDEX
{
    ODOM,
    COMPAS,
    LIDAR
	};
	class Lidar : public AbstractSensor
	{
		public:
			/**
			 *
			 */
			Lidar();
			/**
			 *
			 */
			Lidar( AbstractAgent* aRobot,double stddev);
			/**
			 *
			 */
			virtual ~Lidar();
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() override;


			void setPosition(const Point& aPosition);

			double getDistanceToPoint(const Point& start,const Point& end);
			Point getPointFromAngle(Point lidarPosition,double angle , double distance);
			double getSingleMeasurement(Point lidarPosition,double angleInDegrees);
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}
			
		protected:
		private:
		Point position;
	};

class Compas : public AbstractSensor
	{
		public:
			/**
			 *
			 */
			Compas() = delete;
			/**
			 *
			 */
			Compas( AbstractAgent* aRobot,double stddev);
			/**
			 *
			 */
			virtual ~Compas();

			double getMeasurement(const Point& position,const Point& newPosition);
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() override;
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}
			
		protected:
		private:
		double angle;
		RandomSTD noise;
	};
class OdomSensor : public AbstractSensor
	{
		public:
			/**
			 *
			 */
			OdomSensor();
			/**
			 *
			 */
			OdomSensor( AbstractAgent* aRobot,double stddev);
			/**
			 *
			 */
			virtual ~OdomSensor();
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() override;
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}
			
		protected:
		private:
			Point position;
	};
}
#endif
