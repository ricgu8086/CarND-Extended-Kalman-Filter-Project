#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools
{
public:
	/**
	 * Constructor.
	 */
	Tools();

	/**
	 * Destructor.
	 */
	virtual ~Tools();

	/**
	 * A helper method to calculate RMSE.
	 */
	Eigen::VectorXd CalculateRMSE(
			const std::vector<Eigen::VectorXd> &estimations,
			const std::vector<Eigen::VectorXd> &ground_truth);

	/**
	 * A helper method to calculate Jacobians.
	 */
	Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

	/**
	 * A helper method to convert from polar to cartesian coordinates.
	 */
	static Eigen::VectorXd polar2cartesian(const Eigen::VectorXd& polar);
	/**
	 * A helper method to convert from cartesian to polar coordinates.
	 */
	static Eigen::VectorXd cartesian2polar(const Eigen::VectorXd& cartesian);

};

#endif /* TOOLS_H_ */
