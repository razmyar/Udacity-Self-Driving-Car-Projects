#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
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
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;


    //Measurement matrix - Laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    //Measurement matrix - RADAR
    Hj_ <<  1, 1, 0, 0,
            1, 1, 0, 0,
            1, 1, 1, 1;


    //Initializing state transition function F
    ekf_.F_ = MatrixXd(4, 4);

    ekf_.F_ <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //Initializing state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


    //Initializing the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_) {


        // first measurement
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 0, 0, 0, 0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

            // Convert radar from polar to cartesian coordinates

            float ro = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            float ro_dot = measurement_pack.raw_measurements_(2);

            // x
            ekf_.x_(0) = ro * cos(phi);

            // y
            ekf_.x_(1) = ro * sin(phi);

            // vx
            ekf_.x_(2) = ro_dot * cos(phi);

            // vy
            ekf_.x_(3) = ro_dot * sin(phi);

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // Initialize state.

            ekf_.x_(0) = measurement_pack.raw_measurements_(0);
            ekf_.x_(1) = measurement_pack.raw_measurements_(1);
            ekf_.x_(2) = 0;
            ekf_.x_(3) = 0;


        }

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;


    if (dt > 0.02) {

        float dt_2 = dt * dt;
        float dt_3 = dt_2 * dt;
        float dt_4 = dt_3 * dt;

        //integrate dt into F
        ekf_.F_(0, 2) = dt;
        ekf_.F_(1, 3) = dt;

        //acceleration noise components
        float noise_ax = 9;
        float noise_ay = 9;

        //set the process covariance matrix Q
        ekf_.Q_ <<
                dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
                0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
                dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
                0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;


        ekf_.Predict();
    }
    /**
     * Update
     */


    // Update based on the sensor type
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    } else {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);

    }

}
