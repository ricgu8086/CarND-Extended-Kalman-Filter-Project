#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
		MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict()
{
	/**
	 * predict the state
	 */

	x_ = F_*x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
	/**
	 * update the state by using Kalman Filter equations
	 */

	MatrixXd y, s, k, I;
	I = MatrixXd::Identity(x_.size(), x_.size());

	y = z-H_*x_;

    s = H_*P_*H_.transpose() + R_;
    k = P_*H_.transpose()*s.inverse();
    x_ = x_ + (k*y);
    P_ = (I - k*H_)*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
	/**
	 * update the state by using Extended Kalman Filter equations
	 */

	MatrixXd y, s, k, I;
	I = MatrixXd::Identity(x_.size(), x_.size());

	y = z-x_; // H here is the identity, so don't need to write it

    s = H_*P_*H_.transpose() + R_;
    k = P_*H_.transpose()*s.inverse();
    x_ = x_ + (k*y);
    P_ = (I - k*H_)*P_;
}
