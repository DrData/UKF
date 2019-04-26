#include "KalmanFilterFactory.h"
#include "measurement_package.h"
#include "ukf.h"

//test

using namespace KF;

UKF * KF::KalmanFilterFactory::MakeKalmanFilter(MeasurementPackage::SensorType stype, KFType kfType)
{
	UKF * kfs = NULL;
	switch (stype)
	{
		case MeasurementPackage::LASER:
			if (kfType == eUKF)
				kfs = new UKFLaser(_pSV);
			else if (kfType == KFType::eKF)
				//kfs = new LaserKalmanFilter(_pSV);
				kfs = NULL;
		break;
		case MeasurementPackage::RADAR:
			if (kfType == eUKF)
				kfs = new UKFRadar(_pSV);
			else if (kfType == eKF)
				//kfs = new RadarKalmanFilter(_pSV);
				kfs = NULL;
		break;
	}
	return kfs;
}
