#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

Tools::Tools()
{
}

Tools::~Tools()
{
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
		const vector<VectorXd> &ground_truth)
{
	/**
	 * Calculate the RMSE here.
	 */

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	VectorXd residual;

	for(unsigned int i=0; i<estimations.size(); i++)
	{
		residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();

		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
	/**
	 * Calculate a Jacobian here.
	 */
	MatrixXd Hj(3,4);

	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//check division by zero
	if (px == 0 && py == 0 )
	{
	    cout << "px or py is 0 and can lead to unexpected results."
	    " Please check it." << endl;

	    return Hj;
	}

	//compute the Jacobian matrix
	double mag_sq = px*px + py*py;
	double pow_half = pow(mag_sq, 1.0/2);
	double pow_three_half = pow(mag_sq, 3.0/2);

	Hj << px/pow_half                       , py/pow_half                       , 0          , 0          ,
	      -py/mag_sq                        , px/mag_sq                         , 0          , 0          ,
	      py*(vx*py - vy*px)/pow_three_half , px*(vy*px - vx*py)/pow_three_half , px/pow_half, py/pow_half;

	return Hj;


}

VectorXd Tools::polar2cartesian(const VectorXd& polar)
{
	double rho = polar(0);
	double phi = polar(1);
	double rho_dot = polar(2);

	double px = rho*sin(phi);
	double py = -rho*cos(phi);
	double vx = rho_dot*sin(phi);
	double vy = -rho_dot*sin(phi);

	VectorXd cartesian(4);
	cartesian << px , py, vx, vy;

	return cartesian;
}

VectorXd Tools::cartesian2polar(const VectorXd &cartesian)
{
	double px = cartesian(0);
	double py = cartesian(1);
	double vx = cartesian(2);
	double vy = cartesian(3);

	VectorXd polar = VectorXd(3);

	polar(0) = sqrt(px*px + py*py);
	polar(1) = atan2(py,px);
	polar(2) = (px*vx + py*vy)/max(.0001,polar(0));

	return polar;
}
