#pragma once
#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "StateVector.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace KF
{
	class UKF
	{
	public:
		UKF(StateVector *psv);
		virtual ~UKF() {};

		virtual void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	protected:

		// Abstract Methods to be implemented based on sensor type
		virtual bool InitializeMeasurement(const MeasurementPackage &m) = 0;
		virtual void Convert2MeasurementSpace(const MatrixXd &Xsig_pred, MatrixXd &Zsig_pred)=0;
		virtual void NormalizeAngles(VectorXd& z_diff)=0;


		/* Methods */
		//void GenerateSigmaPoints(MatrixXd * Xsig_out);
		void AugmentedSigmaPoints(MatrixXd &Xsig_out);
		void SigmaPointPrediction(MatrixXd &Xsig_pred, float dt, MatrixXd &Xsig_out);
		void PredictMeanAndCovariance(MatrixXd &Xsig_pred, VectorXd& x_out, MatrixXd& P_out);
		void Update(MatrixXd& Xsig_pred, VectorXd& x_out, MatrixXd& P_out);

		StateVector*_pSV;

		/* Member Variables */
		///* initially set to false, set to true in first call of ProcessMeasurement
		bool is_initialized_;

		///* predicted sigma points matrix
		MatrixXd Xsig_pred_;

		///* time when the state is true, in us
		long long time_us_;

		///* Process noise standard deviation longitudinal acceleration in m/s^2
		double std_a_squared_;

		///* Process noise standard deviation yaw acceleration in rad/s^2
		double std_yawdd_squared_;

		///* Weights of sigma points
		VectorXd weights_;

		///* State dimension
		int n_x_;
		int n_sig_pts_;

		//set measurement dimension, radar can measure r, phi, and r_dot
		int n_z_;

		// Measurement vector -
		VectorXd zm_;

		// Mean predicted measurement
		VectorXd z_pred_;

		// Augmented state dimension
		int n_aug_;
		int n_sig_pts_aug_;

		///* Sigma point spreading parameter
		double lambda_;


		MatrixXd R_;

		// augemented mean state
		VectorXd x_aug_;

		//create augmented state covariance
		MatrixXd P_aug_;

		VectorXd x_diff_;
		VectorXd z_diff_;

		MatrixXd Tc; // GetNZ
		MatrixXd S; // GetNZ

	};


	class UKFLaser : public UKF
	{
	public:
		/**
		* Constructor
		*/
		UKFLaser(StateVector *pSV);

		/**
		* Destructor
		*/
		virtual ~UKFLaser();


	protected:
		UKFLaser();

		virtual bool InitializeMeasurement(const MeasurementPackage &m);
		virtual void NormalizeAngles(VectorXd& z_diff) {};
		virtual void Convert2MeasurementSpace(const MatrixXd & Xsig_pred, MatrixXd & Zsig_pred);


		///* if this is false, laser measurements will be ignored (except for init)
		bool process_data;

		///* Laser measurement noise standard deviation position1 in m
		double std_laspx_;

		///* Laser measurement noise standard deviation position2 in m
		double std_laspy_;

	};

	class UKFRadar : public UKF
	{
	public:
		/**
		* Constructor
		*/
		UKFRadar(StateVector *pSV);

		/**
		* Destructor
		*/
		virtual ~UKFRadar();

		/**
		* ProcessMeasurement
		* @param meas_package The latest measurement data of either radar or laser
		*/
		//virtual void ProcessMeasurement(const MeasurementPackage &meas_package);

	protected:
		UKFRadar();

		virtual bool InitializeMeasurement(const MeasurementPackage &m);
		virtual void Convert2MeasurementSpace(const MatrixXd & Xsig_pred, MatrixXd & Zsig_pred);
		virtual void NormalizeAngles(VectorXd& z_diff);

		///* if this is false, radar measurements will be ignored (except for init)
		bool process_data;

		///* Radar measurement noise standard deviation radius in m
		double std_radr_;

		///* Radar measurement noise standard deviation angle in rad
		double std_radphi_;

		///* Radar measurement noise standard deviation radius change in m/s
		double std_radrd_;

	};
}
