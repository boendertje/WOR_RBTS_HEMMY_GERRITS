#include "MathUtils.hpp"

namespace Utils
{
	/**
	 *
	 */
	/* static */ double MathUtils::toRadians( double aDegrees)
	{
		return aDegrees * PI / 180.0;
	}
	/**
	 *
	 */
	/* static */ double MathUtils::toDegrees( double aRadian)
	{
		return aRadian * 180.0 / PI;
	}
	double MathUtils::normaliseDegrees(double degrees)
	{
		if(degrees > 360)
		{
			degrees -= 360;
		}
		if(degrees < 0)
		{
			degrees += 360;
		}
		return degrees;
	}
} //namespace Utils
