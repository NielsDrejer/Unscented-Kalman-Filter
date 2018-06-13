#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // State dimension
  n_x_ = 5;
    
  // Augmented dimension
  n_aug_ = 7;
    
  // Spreading parameter
  lambda_ = 3 - n_aug_;
    
  // Vector for weights
  weights_ = VectorXd((2 * n_aug_) + 1);
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<((2*n_aug_)+1); i++) {
      double weight = 0.5/(n_aug_+lambda_);
      weights_(i) = weight;
  }

  // Initialize state covariance matrix. Use identity matrix as proposed in lesson
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Initialize radar measurement noise covariance matrix
  R_radar_ = MatrixXd(3, 3); // Radar measurement dimension 3: r, phi and r_dot
  R_radar_ << (std_radr_ * std_radr_), 0,                           0,
              0,                       (std_radphi_ * std_radphi_), 0,
              0,                       0,                           (std_radrd_ * std_radrd_);


  // Initialize laser measurement noise covariance matrix
  R_laser_ = MatrixXd(2, 2); //Laser measurement dimension 2, px and py
  R_laser_ << (std_laspx_ * std_laspx_), 0,
              0,                         (std_laspy_ * std_laspy_);

  nisfile_.open("nis.txt");
}

UKF::~UKF() {
  nisfile_.close();
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    // first measurement
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_[0]; // Range
      double phi = meas_package.raw_measurements_[1]; // Bearing
      double rho_dot = meas_package.raw_measurements_[2]; // Velocity of range

      /* Convert to cartesian */
      double x = rho * cos(phi);
      if( x < 0.0001 ) {
        x = 0.0001;
      }

      double y = rho * sin(phi);
      if( y < 0.0001 ) {
        y = 0.0001;
      }
      
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      // Calculate velocity
      double v = sqrt((vx * vx) + (vy * vy));

      /* Initialize state, no known value for yaw and yaw rate, use 0 */
      x_ << x, y, v, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      /* x and y are measured directly, we cannot know v, yaw and yaw rate, use 0 */
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }

    time_us_ = meas_package.timestamp_;
    
    is_initialized_ = true;

    return;
  }

  // All other measurements
  double dt = ( meas_package.timestamp_ - time_us_ ) / 1000000.0; /* dt - expressed in seconds */

  time_us_ = meas_package.timestamp_;

  // Prediction step
  Prediction(dt);

  // Measurement step
  if ( (meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_ ) {
    // Radar updates
//    cout << "Measurement Radar" << endl;
    UpdateRadar(meas_package);
  } else if ( (meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_ ){
    // Laser updates
//    cout << "Measurement Laser" << endl;
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Step 1. Generate Augmented Sigma points

  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
    
  // Augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
  // Augmented sigma points matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, ((2 * n_aug_) + 1));
    
  // Create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0; // Mean is zero
  x_aug(6) = 0; // Mean is zero
    
  // Create augmented covariance matrix
  MatrixXd Q = MatrixXd(2, 2);
  Q << (std_a_ * std_a_), 0,
        0,                (std_yawdd_ * std_yawdd_);
    
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;    
  // cout << "P_aug = " << endl << P_aug << endl;
      
  // Create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
    
  // Create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // Step 2: Predict the Sigma points

  // Create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, ((2 * n_aug_) + 1));
  double delta_t_fac = (delta_t * delta_t)/2;

  for(int i = 0; i < Xsig_aug.cols(); i++) {
    double px = Xsig_aug.col(i)(0);
    double py = Xsig_aug.col(i)(1);
    double v = Xsig_aug.col(i)(2);
    double yaw = Xsig_aug.col(i)(3);
    double yaw_d = Xsig_aug.col(i)(4);
    double nu_a = Xsig_aug.col(i)(5);
    double nu_yawdd = Xsig_aug.col(i)(6);
        
    // Check value of psi_dot (5th element in each column)
    if(fabs(yaw_d) > 0.001) {
      Xsig_pred_.col(i)(0) = px + ((v/yaw_d) * (sin(yaw + yaw_d * delta_t) - sin(yaw))) + (delta_t_fac * cos(yaw) * nu_a);
      Xsig_pred_.col(i)(1) = py + ((v/yaw_d) * (-cos(yaw + yaw_d * delta_t) + cos(yaw))) + (delta_t_fac * sin(yaw) * nu_a);
    }
    else {
      Xsig_pred_.col(i)(0) = px + (v*cos(yaw) * delta_t) + (delta_t_fac * cos(yaw) * nu_a);
      Xsig_pred_.col(i)(1) = py + (v*sin(yaw) * delta_t) + (delta_t_fac * cos(yaw) * nu_a);
    }
        
    Xsig_pred_.col(i)(2) = v + 0 + (delta_t * nu_a);
    Xsig_pred_.col(i)(3) = yaw + (yaw_d * delta_t) + (delta_t_fac * nu_yawdd);
    Xsig_pred_.col(i)(4) = yaw_d + 0 + (delta_t * nu_yawdd);
  }

  // Step 3: Calculate predicted mean covariance from predicted sigma points

  // Calculate predicted state mean
  x_ = (Xsig_pred_ * weights_).rowwise().sum();
    
  // Calculate predicted state covariance matrix
  P_.fill(0.0);
  for(int i = 0; i < ((2 * n_aug_) + 1); i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
    // angle normalization
    NormalizeAngle(x_diff(3));
        
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // Step 1 Predict the Laser measurement. Laser measures x and y directly

  int n_z = 2;

  // Matrix for sigma points in measurement space
  MatrixXd Zsig = Xsig_pred_.block(0,0, n_z, ((2 * n_aug_) + 1)); // Use px and py from Sigma points directly

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  // Calculate mean predicted measurement
  z_pred.fill(0.0);
  z_pred = (Zsig * weights_).rowwise().sum();

  // Calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < ((2 * n_aug_) + 1); i++) {  //iterate over sigma points
    // state difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
        
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement covariance noise
  S = S + R_laser_;

  // Step 2 Update state
  // The incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
    
  //calculate cross correlation matrix
  Tc.fill(0.0);
    
  for (int i = 0; i < ((2 * n_aug_) + 1); i++) {  //iterate over sigma points
    // State space difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
    // Measurement space difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
                
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
    
  // Calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Update state mean and covariance matrix
  // Residual
  VectorXd z_diff = z - z_pred;

  x_ = x_ + K * z_diff;
    
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for current step
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  nisfile_ << "NIS Laser = " << NIS_laser_ << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // Step 1 Predict the radar measurement

  // Measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // Matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, ((2 * n_aug_) + 1));
    
  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
    
  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
    
  // Transform sigma points into measurement space
  for (int i = 0; i < ((2*n_aug_) + 1); i++) {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
        
    double rho = sqrt(px*px + py*py);
    double phi = atan2(py, px);
        
    // Angle normalization
    NormalizeAngle(phi);
        
    double rho_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / rho;
        
    Zsig(0,i) = rho;
    Zsig(1,i) = phi;
    Zsig(2,i) = rho_dot;
  }
    
  // Calculate mean predicted measurement
  z_pred.fill(0.0);
  z_pred = (Zsig * weights_).rowwise().sum();

  // Calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < ((2 * n_aug_) + 1); i++) {  //iterate over sigma points
    // state difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
        
    // Angle normalization
    NormalizeAngle(z_diff(1));
        
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  // Add measurement covariance noise
  S = S + R_radar_;
    
  // Step 2 Update state

  // The incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
    
  //calculate cross correlation matrix
  Tc.fill(0.0);
    
  for (int i = 0; i < ((2 * n_aug_) + 1); i++) {  //iterate over sigma points
    // State space difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
    // Measurement space difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
        
    // Angle normalization 1
    NormalizeAngle(x_diff(3));

    // Angle normalization 2
    NormalizeAngle(z_diff(1));
        
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
    
  // Calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
    
  // Update state mean and covariance matrix
  // Residual
  VectorXd z_diff = z - z_pred;
    
  // Angle normalization
  NormalizeAngle(z_diff(1));

  x_ = x_ + K * z_diff;
    
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for current step
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  nisfile_ << "NIS Radar = " << NIS_radar_ << endl;
}

/* Helper function to ensure an angle is between -M_PI and M_PI */
void UKF::NormalizeAngle(double& angle) {
    // angle normalization
    while (angle>M_PI) angle-=2.*M_PI;
    while (angle<-M_PI) angle+=2.*M_PI;
}
