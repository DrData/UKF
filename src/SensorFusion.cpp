
#include "SensorFusion.h"
#include "measurement_package.h"
#include "ukf.h"
#include "KFEnums.h"
#include "KalmanFilterFactory.h"
#include <map>


// State vector (px, py, v, yaw, yaw_dot)
#define N_STATE_VECTOR_DIMENSION 5

// Add longitudinal and yaw acceleration noise
#define N_AUGMENTED_STATE_VECTOR_DIMENSION 7

using namespace KF;
using namespace std;

SensorFusion::SensorFusion(KFType kftype)
	:theState_(N_STATE_VECTOR_DIMENSION)
{
	StateVector *pTheState = &theState_;

	KalmanFilterFactory KFFactory(pTheState);

	mapSensorsType2KF_.insert(pair<MeasurementPackage::SensorType, UKF *>(MeasurementPackage::LASER, KFFactory.MakeKalmanFilter(MeasurementPackage::LASER, kftype)));

	mapSensorsType2KF_.insert(pair<MeasurementPackage::SensorType, UKF *>(MeasurementPackage::RADAR, KFFactory.MakeKalmanFilter(MeasurementPackage::RADAR, kftype)));

}

SensorFusion::~SensorFusion()
{

}

void SensorFusion::ProcessMeasurement(const MeasurementPackage &mp)
{
	mapSensorsType2KF_[mp.sensor_type_]->ProcessMeasurement(mp);
}



