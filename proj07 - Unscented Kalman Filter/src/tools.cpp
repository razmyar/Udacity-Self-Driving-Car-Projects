#include "tools.h"
#include <iostream>
#include <iomanip>


using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rmse = VectorXd(4);
    rmse << 0, 0, 0, 0;

    // Validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size() ||
        estimations.size() == 0) {
        std::cout << "[ERROR] Invalid inputs to CalculateRMSE function." << std::endl;
        return rmse;
    }

    // accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        //cw multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //mean
    rmse = rmse / estimations.size();

    //squared root
    rmse = rmse.array().sqrt();

    if (rmse(0) > .09 || rmse(1) > .10 ||
        rmse(2) > .40 || rmse(3) > .30) {
        std::cout << std::fixed;
        std::cout << std::setprecision(2);
        std::cout << "RMSE is larger than limit:\nCurrent RMSE: ["
                  << (rmse(0)) << ", " << (rmse(1)) << ", "
                  << (rmse(2)) << ", " << (rmse(3)) << "] \n"
                  << "RMSE Limit: "
                  << "[.09, .10, .40, .30]\n\n";
    } else
        std::cout << "RMSE satisfied!" << "\n";


    //return rmse
    return rmse;
}