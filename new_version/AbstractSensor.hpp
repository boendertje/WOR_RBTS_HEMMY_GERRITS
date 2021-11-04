#ifndef ABSTRACTSENSOR_HPP_
#define ABSTRACTSENSOR_HPP_

#include "Config.hpp"

#include "Thread.hpp"
#include "ModelObject.hpp"
#include "worSrc/ignore/Matrix.hpp"
namespace Model
{
	class AbstractAgent;
	typedef std::shared_ptr< AbstractAgent > AbstractAgentPtr;


	/**
	 *
	 */
	class AbstractStimulus
	{
		public:
			/**
			 *
			 */
			virtual ~AbstractStimulus()
			{
			}
	};
	// class AbstractStimulus
	/**
	 *
	 */
	class AbstractPercept
	{
		public:
			/**
			 *
			 */
			virtual ~AbstractPercept()
			{
			}
	};
	class DistanceStimulus : public AbstractStimulus
	{
		public:
			DistanceStimulus( 	double anAngle,
								double aDistance) :
				angle(anAngle),
				distance( aDistance)
		{
		}
		double angle;
		double distance;
	};

	class LidarStimulus : public AbstractStimulus
	{
		public:
			LidarStimulus(const Matrix<double,180,1>& aDistances):distances(aDistances)
			{}
		Matrix<double,180,1> distances;
	};
	// class DistanceStimulus
	class LidarPercept : public AbstractPercept
	{
		public:
			LidarPercept( const LidarStimulus& aDLidarStimulus) :
				distances(aDLidarStimulus.distances)
		{
		}
		LidarPercept(const Matrix<double,180,1>& aDistances) :
			distances(aDistances)
		{
		}
		double determineWeight(const std::shared_ptr<LidarPercept>& measurement)
		{
			double difference = 0;
			double range = 1024;
			double maxDif = range * static_cast<double>(distances.getRows());
			for(size_t i = 0; i < distances.getRows();++i)
			{
				difference += fabs(measurement->distances[i][0] - distances[i][0]);
			}
			difference = maxDif - difference;
			difference = difference/maxDif;
			return difference;
		}
		
		Matrix<double,180,1> distances;
	};
	/**
	 *
	 */
	class DistancePercept : public AbstractPercept
	{
		public:
			DistancePercept( const DistanceStimulus& aDistanceStimulus) :
				angle(aDistanceStimulus.angle),
				distance( aDistanceStimulus.distance)
		{
		}
		DistancePercept(double anAngle,
						double aDistance) :
			angle(anAngle),
			distance( aDistance)
		{
		}
		double angle;
		double distance;
		double determineWeight(double angleMeasurement)
		{
			double difference = angle - angleMeasurement;
			double range = 360;
			double maxDif = range;
			
			difference = maxDif - difference;
			difference = difference/maxDif;
			return difference;
		}
	};
	// class AbstractPercept

	class AbstractSensor : public ModelObject
	{
		public:
			/**
			 *
			 */
			AbstractSensor();
			/**
			 *
			 */
			AbstractSensor( AbstractAgent* anAgent,double stddev = 0);
			/**
			 *
			 */
			/**
			 *
			 */
			virtual ~AbstractSensor();
			/**
			 * A sensor reads 10 stimuli/second (it sleeps for 100 ms) by default
			 */
			virtual void setOn( unsigned long aSleepTime = 100);
			/**
			 *
			 */
			virtual void setOff();
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() = 0;
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractPercepts) const = 0;
			/**
			 *
			 */
			virtual void sendPercept( std::shared_ptr< AbstractPercept > anAbstractPercept);
			/**
			 *
			 */
			virtual void run( unsigned long aSleepTime);
			/**
			 *
			 */
			virtual void attachAgent( AbstractAgent* anAgent);
			/**
			 *
			 */
			virtual void detachAgent();
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const;
			//@}
			double getStdDev() const;

		protected:
			AbstractAgent* agent;
			bool running;
			std::thread sensorThread;
			mutable std::recursive_mutex sensorMutex;
			double stddev;

		private:
	};
// class AbstractSensor
}// namespace Model

#endif // ABSTRACTSENSOR_HPP_
