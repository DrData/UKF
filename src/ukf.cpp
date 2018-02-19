#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "StateVector.h"

using namespace KF;
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define TWOPI 6.28318530718

inline double NormalizeAngle(double ang)
{
	return fmod(ang, TWOPI);
}


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF(StateVector *psv) : _pSV(psv)
{
	// Process noise standard deviation longitudinal acceleration in m/s^2
	int std_a_ = 30;

	// Process noise standard deviation yaw acceleration in rad/s^2
	int std_yawdd_ = 30;
	//std_a_squared_ = 0.2*0.2;
	//std_yawdd_squared_ = 0.2*0.2;

	//Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_squared_ = std_a_* std_a_;
	//Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_squared_ = std_yawdd_ * std_yawdd_;

	///* State dimension
	n_x_ = psv->size();
	n_sig_pts_ = 2 * n_x_ + 1;

	n_z_ = 0;

	//* Augmented state dimension
	n_aug_ = n_x_+2;

	n_sig_pts_aug_ = 2 * n_aug_ + 1;

	lambda_ = 3 - n_x_;

	// Initialize temporary variables
	double wt = 0.5 / (n_aug_ + lambda_);
	weights_ = VectorXd::Constant(n_sig_pts_aug_,wt);
	weights_(0) = lambda_ / (lambda_ + n_aug_);

	Xsig_pred_ = MatrixXd(n_x_, n_sig_pts_aug_);

	x_aug_ = VectorXd(n_aug_);
	P_aug_ = MatrixXd(n_aug_, n_aug_);

	S = MatrixXd(n_x_, n_x_);
}


void UKF::ProcessMeasurement(const MeasurementPackage &meas_package)
{
	// Processes the measurement based on sensor type
	// in this derived method
	// Handles first measurement and loads a vector with the measurement.
	if (InitializeMeasurement(meas_package)) return;

	float delta_t = (meas_package.timestamp_ - _pSV->previous_timestamp_) / 1000000.0;
	_pSV->previous_timestamp_ = meas_package.timestamp_;

	// Generate Sigma Points
	MatrixXd Xsig_pts = MatrixXd::Zero(n_aug_, n_sig_pts_aug_);
	AugmentedSigmaPoints(Xsig_pts);

	// Predict Points
	SigmaPointPrediction(Xsig_pts, delta_t, Xsig_pred_);

	VectorXd x_pred_mean = VectorXd(n_x_);
	MatrixXd P_pred_mean = MatrixXd(n_x_, n_x_);
	PredictMeanAndCovariance(Xsig_pred_, x_pred_mean, P_pred_mean);

	// Update the state based on difference between Predicted Measurement and actual Measurement
	Update(Xsig_pred_, x_pred_mean, P_pred_mean);

}


/**
* @param {MeasurementPackage} meas_package The latest measurement data of
* either radar or laser.
*/
/**
* Predicts sigma points, the state, and the state covariance matrix.
* @param {double} delta_t the change in time (in seconds) between the last
* measurement and this one.
*/
//void UKFLaser::Prediction(float delta_t) {
	/**
	TODO:

	Complete this function! Estimate the object's location. Modify the state
	vector, x_. Predict sigma points, the state, and the state covariance matrix.
	*/
	// Process Model
//}

/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} meas_package
*/
//void UKFLaser::Update(const MeasurementPackage &meas_package) {
		/**
	TODO:

	Complete this function! Use lidar data to update the belief about the object's
	position. Modify the state vector, x_, and covariance, P_.

	You'll also need to calculate the lidar NIS.
	*/
//}

/**
* Updates the state and the state covariance matrix using a radar measurement.
* @param {MeasurementPackage} meas_package
*/

/*
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

	//define spreading parameter
	lambda_ = 3 - n_x_;

	//create sigma point matrix
	MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

	//calculate square root of P_
	MatrixXd A = _pSV->P_.llt().matrixL();

	//set first column of sigma point matrix
	Xsig.col(0) = _pSV->x_;

	//set remaining sigma points
	for (int i = 0; i < n_x_; i++)
	{
		Xsig.col(i + 1) = _pSV->x_ + sqrt(lambda_ + n_x_) * A.col(i);
		Xsig.col(i + 1 + n_x_) = _pSV->x_ - sqrt(lambda_ + n_x_) * A.col(i);
	}

	*Xsig_out = Xsig;
}
*/

void UKF::AugmentedSigmaPoints(MatrixXd& Xsig_out)
{
	//create augmented mean state
	x_aug_.head(n_x_) = _pSV->x_;
	x_aug_(5) = 0;
	x_aug_(6) = 0;

	//create augmented covariance matrix
	P_aug_.fill(0.0);
	P_aug_.topLeftCorner(n_x_, n_x_) = this->_pSV->P_;
	P_aug_(5, 5) = std_a_squared_;
	P_aug_(6, 6) = std_yawdd_squared_;

	//create square root matrix
	MatrixXd L = P_aug_.llt().matrixL();

	//create augmented sigma points
	Xsig_out.col(0) = x_aug_;
	for (int i = 0; i< n_aug_; i++)
	{
		Xsig_out.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_out.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
	}

	//std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
}

/*
* Propagate Sigma Points with Solution to equations of motion.
*/
void UKF::SigmaPointPrediction(MatrixXd& Xsig_aug, float delta_t, MatrixXd& Xsig_pred)
{
	//predict sigma points
	for (int i = 0; i< n_sig_pts_aug_; i++)
	{
		//extract values for better readability
		double p_x  = Xsig_aug(0, i);
		double p_y  = Xsig_aug(1, i);
		double v    = Xsig_aug(2, i);
		double yaw  = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);

		//predicted state values
		double px_p = p_x;
		double py_p = p_y;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p += v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
			py_p += v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
		}
		else {
			px_p += v * delta_t*cos(yaw);
			py_p += v * delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;

		double dt2 = delta_t * delta_t;
		//add noise
		px_p = px_p + 0.5*nu_a*dt2 * cos(yaw);
		py_p = py_p + 0.5*nu_a*dt2 * sin(yaw);
		v_p = v_p + nu_a * delta_t;

		yaw_p = yaw_p + 0.5*nu_yawdd*dt2 + nu_yawdd * delta_t;
		yaw_p = NormalizeAngle(yaw_p);

		//write predicted sigma point into right column
		Xsig_pred(0, i) = px_p;
		Xsig_pred(1, i) = py_p;
		Xsig_pred(2, i) = v_p;
		Xsig_pred(3, i) = yaw_p;
		Xsig_pred(4, i) = yawd_p;
	}
}

void UKF::PredictMeanAndCovariance(MatrixXd &Xsig_pred, VectorXd& xMean, MatrixXd& P)
{
	//predicted state mean
	xMean = Xsig_pred * weights_; // (n_x_, n_aug_) * (n_aug_)

	//predicted state covariance matrix
	P.fill(0.0);
	//iterate over sigma points
	int nsigmapts = P.cols();
	for (int i = 0; i < n_sig_pts_aug_ ; i++) {
		// state difference
		x_diff_ = Xsig_pred.col(i) - xMean;

		x_diff_(3) = NormalizeAngle(x_diff_(3));

		P = P + weights_(i) * x_diff_ * x_diff_.transpose();
	}
}

/*
* Update
* Based on predicted sigma points
* 1) Compute the predicted in measurement space
* 2) Combine with actual measurment via Kalman Filter algorithm
* 3) Update the state vector and covariance
* Input:
*	Xsig_pred - predicted sigma points
*/
void UKF::Update(MatrixXd& Xsig_pred, VectorXd& x_pred, MatrixXd& P_pred)
{

	/*
	* Transform to measurement space fomr the predicted sigma points in state space
	*/
	MatrixXd Zsig_pred = MatrixXd(n_z_, n_sig_pts_aug_);

	// Convert to measurment is implemented by derived method specify to sensor type
	Convert2MeasurementSpace(Xsig_pred, Zsig_pred);

	// Compute the mean of the predicted measurement based on predict sigma points
	z_pred_ = Zsig_pred * weights_;

	// Compute the innovation covariance of the predicted measurement based on predict sigma points
	S = MatrixXd::Zero(n_z_, n_z_);
	for (int i = 0; i < n_sig_pts_aug_; i++)
	{
		// Compute Residual
		VectorXd z_diff = Zsig_pred.col(i) - z_pred_;

		NormalizeAngles(z_diff);

		// Weight contribution for current sigma point
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	// Add measurement noise covariance matrix to innovation
	S = S + R_;

	/*
	* Now Update the state by fusing predicted with measurement
	*/
	//create matrix for cross correlation Tc
	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < n_sig_pts_aug_; i++)
	{
		//residual
		z_diff_ = Zsig_pred.col(i) - z_pred_;

		// Angle normalization in measurement space
		NormalizeAngles(z_diff_);

		// state difference
		x_diff_ = Xsig_pred.col(i) - x_pred;

		//angle normalization of state angle variables
		x_diff_(3) = NormalizeAngle(x_diff_(3));

		//Tc = MatrixXd(n_x_, n_z_); // GetNZ
		Tc = Tc + weights_(i) * x_diff_ * z_diff_.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	// Residual between the actual measurement and the predicted measurement
	z_diff_ = zm_ - z_pred_;

	// Angle normalization
	NormalizeAngles(z_diff_);

	//update state mean and covariance matrix
	_pSV->x_ = x_pred + K * z_diff_;
	_pSV->P_ = P_pred - K * S*K.transpose();

}


/**  Radar UKF Class

*/


/*
//DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
// Radar measurement noise standard deviation radius in m
std_radr_ = 0.3;

// Radar measurement noise standard deviation angle in rad
std_radphi_ = 0.03;

// Radar measurement noise standard deviation radius change in m/s
std_radrd_ = 0.3;
*/
UKFRadar::UKFRadar(StateVector *pSV) : UKF(pSV), std_radr_(0.3),
std_radphi_(0.03), std_radrd_(0.3)
{
	n_z_ = 3;

	//add measurement noise covariance matrix
	VectorXd tmp(n_z_);
	tmp << std_radr_ * std_radr_, std_radphi_*std_radphi_, std_radrd_*std_radrd_;
	R_ = tmp.asDiagonal();

	zm_ = VectorXd(n_z_);
	z_pred_ = VectorXd(n_z_);
	Tc = MatrixXd(n_x_, n_z_);
}

bool UKFRadar::InitializeMeasurement(const MeasurementPackage &meas_package)
{
	if (!_pSV->is_initialized_)
	{
		double r = meas_package.raw_measurements_(0);
		double theta = meas_package.raw_measurements_(1);
		double rdot = meas_package.raw_measurements_(2);

		double px = r * cos(theta);
		double py = r * sin(theta);
		double v = rdot;
		double phi = theta;  // yaw rotation about axis perpendicular to plane
		double phidot = 0;

		this->_pSV->x_ << px, py, v, phi, phidot;
		this->_pSV->previous_timestamp_ = meas_package.timestamp_;
		_pSV->is_initialized_ = true;
		return true;
	}
	else
	{
		zm_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);
		return false;
	}
}

void UKFRadar::Convert2MeasurementSpace(const MatrixXd &Xsig_pred, MatrixXd &Zsig_pred)
{
	// Transform predicted sigma points into measurement space
	// 2n+1 sigma points
	for (int i = 0; i < n_sig_pts_aug_; i++)
	{
		// Extract values for better readibility
		double p_x = Xsig_pred(0, i);
		double p_y = Xsig_pred(1, i);
		double v = Xsig_pred(2, i);
		double yaw = Xsig_pred(3, i);
		double yawdot = Xsig_pred(4, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		Zsig_pred(0, i) = sqrt(p_x*p_x + p_y * p_y);//r
													//Zsig_pred(1, i) = NormalizeAngle(atan2(p_y, p_x));					//phi
		Zsig_pred(1, i) = yaw;					//phi
		Zsig_pred(2, i) = (p_x*v1 + p_y * v2) / sqrt(p_x*p_x + p_y * p_y);   //r_dot
	}

}

void UKFRadar::NormalizeAngles(VectorXd& z_diff)
{
	z_diff(1) = NormalizeAngle(z_diff(1));
}


UKFRadar::~UKFRadar()
{

}

/**  Laser UKF Class

*/
UKFLaser::UKFLaser(StateVector *pSV) : UKF(pSV)
{
	n_z_ = 2;

	//DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	//add measurement noise covariance matrix
	VectorXd tmp(n_z_);
	tmp << std_laspx_ * std_laspx_, std_laspy_*std_laspy_;
	R_ = tmp.asDiagonal();

	zm_ = VectorXd(n_z_);
	z_pred_ = VectorXd(n_z_);
	Tc = MatrixXd(n_x_, n_z_);
}

UKFLaser::~UKFLaser()
{

}

void UKFLaser::Convert2MeasurementSpace(const MatrixXd &Xsig_pred, MatrixXd &Zsig_pred)
{
	// Transform predicted sigma points into measurement space
	// 2n+1 sigma points
	for (int i = 0; i < n_sig_pts_aug_; i++)
	{
		// Extract values for better readibility
		double p_x = Xsig_pred(0, i);
		double p_y = Xsig_pred(1, i);

		// measurement model
		Zsig_pred(0, i) = p_x;
		Zsig_pred(1, i) = p_y;
	}
}



bool UKFLaser::InitializeMeasurement(const MeasurementPackage &meas_package)
{
	if (!_pSV->is_initialized_)
	{
		double px = meas_package.raw_measurements_(0);
		double py = meas_package.raw_measurements_(1);
		double v = 0;
		double yaw = 0;
		double yaw_dot = 0;
		_pSV->x_ << px, py, v, yaw, yaw_dot;
		_pSV->previous_timestamp_ = meas_package.timestamp_;
		_pSV->is_initialized_ = true;
		return true;
	}
	else
	{
		zm_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);
	}
	return false;

}



