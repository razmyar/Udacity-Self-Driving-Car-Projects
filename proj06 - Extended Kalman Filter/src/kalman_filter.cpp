#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {

    // Predict the state

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;


}

void KalmanFilter::Update(const VectorXd &z) {

    // update the state by using the Kalman Filter equations


    VectorXd H_prime = H_ * x_;
    VectorXd y = z - H_prime;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    // new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    // Update the state by using Extended Kalman Filter equations
    float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    float phi = atan2(x_(1), x_(0));
    float rho_dot;


    if (rho < 0.0001) {
        rho = 0.0001;
    }
    rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

    VectorXd H_prime(3);
    H_prime << rho, phi, rho_dot;


    VectorXd y = z - H_prime;
    
    // Normalize the angle
    while (y(1) > M_PI) {
        y(1) -= 2 * M_PI;
    }
    while (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
    }

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;


}
