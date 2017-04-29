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

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if (px == 0 || py == 0 )
	{
	    cout << "px or py is 0 and can lead to unexpected results."
	    " Please check it." << endl;

	    return Hj;
	}

	//compute the Jacobian matrix
	float mag_sq = px*px + py*py;
	float pow_half = pow(mag_sq, 1.0/2);
	float pow_three_half = pow(mag_sq, 3.0/2);

	Hj << px/pow_half                       , py/pow_half                       , 0          , 0          ,
	      -py/mag_sq                        , px/mag_sq                         , 0          , 0          ,
	      py*(vx*py - vy*px)/pow_three_half , px*(vy*px - vx*py)/pow_three_half , px/pow_half, py/pow_half;

	return Hj;


}

VectorXd Tools::polar2cartesian(const VectorXd& polar)
{
	float rho = polar(0);
	float phi = polar(1);
	float rho_dot = polar(2);

	float px = rho*sin(phi);
	float py = -rho*cos(phi);
	float vx = rho_dot*sin(phi);
	float vy = -rho_dot*sin(phi);

	VectorXd cartesian(4);
	cartesian << px , py, vx, vy;

	return cartesian;
}
