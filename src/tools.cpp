#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
    */

  VectorXd rmse;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	rmse = VectorXd::Zero(estimations[0].size());

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

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

VectorXd Tools::DeriveStateFromRadarMeas(const VectorXd &meas) {

	/**
	 * Calculate state vector from radar measurement.
	 */
	VectorXd x = VectorXd::Zero(5);

	float rho = meas(0);
	float phi = meas(1);

	x(0) = rho * cos(phi);
	x(1) = - rho * sin(phi);
	return x;
}

VectorXd Tools::DeriveStateFromLaserMeas(const VectorXd &meas) {

	/**
	 * Calculate state vector from laser measurement.
	 */
	VectorXd x = VectorXd::Zero(5);
	x(0) = meas(0);
	x(1) = meas(1);
	return x;
}
