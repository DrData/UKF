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
	VectorXd rsme(4);
	rsme.fill(0);

	long n = estimations.size();
	if (n < 3 || n != ground_truth.size())
	{
		return rsme;
	}

	for (int i = 0; i < n; i++)
	{
		VectorXd xest = estimations[i];
		VectorXd xlabel = ground_truth[i];

		VectorXd dif = xest - xlabel;
		dif = dif.array() * dif.array();

		rsme += dif;
	}

	// Get average of squared differences
	rsme /= (double)n;

	rsme = rsme.array().sqrt();
	return rsme;
}