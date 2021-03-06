#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "logging.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    ReInit();
}

void FusionEKF::ReInit() {
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

    /**
     * TODO: Finish initializing the FusionEKF.
     * TODO: Set the process and measurement noises
     */
    // create a 4D state vector, we don't know yet the values of the x state
    ekf_.x_ = VectorXd(4);

    // state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


    // measurement matrix
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    // the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //acc noise
    noise_ax = 9;
    noise_ay = 9;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() = default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_) {
        /**
         * TODO: Initialize the state ekf_.x_ with the first measurement.
         * TODO: Create the covariance matrix.
         * You'll need to convert radar from polar to cartesian coordinates.
         */

        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            Logging::logging("Init kalman with radar data", Logging::DEBUG);
            // meas_rho meas_phi meas_rho_dot
            ekf_.x_ = Tools::TransformPolar2Cart(measurement_pack.raw_measurements_);

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            Logging::logging("Init kalman with laser data", Logging::DEBUG);
            ekf_.x_ << measurement_pack.raw_measurements_[0],
                    measurement_pack.raw_measurements_[1],
                    0,
                    0;
        }

        // Store time
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */

    /**
     * TODO: Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * TODO: Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    // Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
    Logging::logging("Predict", Logging::DEBUG);
    ekf_.Predict();

    /**
     * Update
     */

    /**
     * TODO:
     * - Use the sensor type to perform the update step.
     * - Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        Logging::logging("Calculate Jacobian radar", Logging::DEBUG);
        ekf_.R_ = R_radar_;
        ekf_.H_ = Tools::CalculateJacobian(ekf_.x_);
        Logging::logging("Update EFK radar", Logging::DEBUG);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        Logging::logging("Update EFK laser", Logging::DEBUG);
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // logging the output
    //cout << "x_ = " << ekf_.x_ << endl;
    //cout << "P_ = " << ekf_.P_ << endl;
}
