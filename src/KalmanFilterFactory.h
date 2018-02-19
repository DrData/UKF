#pragma once
#include "StateVector.h"
#include "KFEnums.h"
#include "UKF.h"
#include "measurement_package.h"

namespace KF
{

	// Creates the requested KTSensor class (laser or radar)
	class KalmanFilterFactory
	{
	public:
		KalmanFilterFactory(StateVector *pSV)
		{
			_pSV = pSV;
		};

		UKF * MakeKalmanFilter(MeasurementPackage::SensorType stype, KFType kfType);

	private:
		StateVector * _pSV;
	};
}