/*
 * LaserDistanceSensor.hpp
 *
 *  Created on: 15 Oct 2012
 *      Author: jkr
 */

#ifndef LASERDISTANCESENSOR_HPP_
#define LASERDISTANCESENSOR_HPP_

#include "Config.hpp"

#include "AbstractSensor.hpp"
#include "Point.hpp"

namespace Model
{
	/**
	 *
	 */
	
	//	class DistancePercept

	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;

	/**
	 *
	 */
	class LaserDistanceSensor : public AbstractSensor
	{
		public:
			/**
			 *
			 */
			LaserDistanceSensor();
			/**
			 *
			 */
			LaserDistanceSensor( Robot* aRobot);
			/**
			 *
			 */
			virtual ~LaserDistanceSensor();
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus();
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const;
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
		protected:
		private:

	};

	
} // namespace Model
#endif /* LASERDISTANCESENSOR_HPP_ */
