#pragma once
#include "measurement_package.h"
#include "ukf.h"
#include "KFEnums.h"
#include <map>
namespace KF
{
	class SensorFusion
	{
	public:
		SensorFusion(KFType kftype);
		virtual ~SensorFusion();
		void ProcessMeasurement(const MeasurementPackage& meas_package);

		std::map<MeasurementPackage::SensorType, UKF *> mapSensorsType2KF_;
		StateVector theState_;
	};

}