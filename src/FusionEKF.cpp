#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
				0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0		, 0,
				0	, 0.0009, 0,
				0	, 0		, 0.09;

	/**
	 * Finish initializing the FusionEKF.
	 * Set the process and measurement noises
	 */

	H_laser_ = MatrixXd::Zero(2,4);
	H_laser_(0,0) = 1;
	H_laser_(1,1) = 1;

	Hj_ = MatrixXd::Zero(3,4); // As jacobian of H is dependent of the state, it won't be full configured here

	MatrixXd Q =  MatrixXd::Zero(4,4); // As process noise Q is dependent of delta T, it won't be full configured here

	MatrixXd P(4, 4);
	P << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1000, 0,
		 0, 0, 0, 1000;

	VectorXd x(4);
	x = VectorXd::Zero(4);

	MatrixXd F = MatrixXd::Zero(4,4);

	ekf_.Init(x, P, F, H_laser_, R_laser_, Q);

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF()
{
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{

	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_)
	{
		/**
		 * Initialize the state ekf_.x_ with the first measurement.
		 * Create the covariance matrix.
		 * Remember: you'll need to convert radar from polar to cartesian coordinates.
		 */
		// first measurement
		cout << "EKF: " << endl;

		this->ekf_.x_ = VectorXd(4);

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		{
			/**
			 Convert radar from polar to cartesian coordinates and initialize state.
			 */
			VectorXd cartesian = tools.polar2cartesian(measurement_pack.raw_measurements_);
			this-> ekf_.x_ << cartesian[0], cartesian[1], 0, 0;
			/*
			 * Seems a bit strange not to use measured velocities to initialize the state.
			 * However, from the lectures:
			 * "Although radar gives velocity data in the form of the range rate rho_dot,
			 * a radar measurement does not contain enough information to determine
			 * the state variable velocities vx and vy. You can, however, use the
			 * radar measurements rho and phi to initialize the state variable locations px and py."
			 */
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		{
			/**
			 Initialize state.
			 */
			//set the state with the initial location and zero velocity
			this->ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		}

		this->previous_timestamp_ = measurement_pack.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	/**
	 * Update the state transition matrix F according to the new elapsed time.
	 - Time is measured in seconds.
	 * Update the process noise covariance matrix Q.
	 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	 */

	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - this->previous_timestamp_) / 1000000.0;	//dt - expressed in seconds

	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;


	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	//set the process covariance matrix Q
	float noise_ax = 9, noise_ay = 9;

	ekf_.Q_ <<  dt_4/4*noise_ax	, 0					, dt_3/2*noise_ax	, 0,
			   	0				, dt_4/4*noise_ay	, 0					, dt_3/2*noise_ay,
				dt_3/2*noise_ax	, 0					, dt_2*noise_ax		, 0,
				0				, dt_3/2*noise_ay	, 0					, dt_2*noise_ay;


	ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	/**
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	 */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
	{
		// Radar updates
		VectorXd z = tools.polar2cartesian(measurement_pack.raw_measurements_);

		// Linearize measurement function H
		Hj_ = tools.CalculateJacobian(ekf_.x_);

		//setup radar matrices
		ekf_.H_ = Hj_;
		ekf_.R_ = R_radar_;

		ekf_.UpdateEKF(z);
	}
	else //MeasurementPackage::LASER
	{
		// Laser updates
		VectorXd z = measurement_pack.raw_measurements_;

		//setup laser matrices
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;

		ekf_.Update(z);
	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
