#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

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
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/8;
  
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
    
  is_initialized_ = false;

  //state vector 
  x_ << 0., //px
        0., //py
        0., //v
        0., //psi
        0.; //psidot
        
  //covariance matrix P
  P_ << 1.,0.,0.,0.,0.,
       0.,1.,0.,0.,0.,
       0.,0.,1.,0.,0.,
       0.,0.,0.,1.,0.,
       0.,0.,0.,0.,1;
  
  time_us_ = 0;
  
  //state vector size
  n_x_ = x_.size();
  
  //augmented state vector size (state vector + process noise components)
  n_aug_ = n_x_ + 2;
  
  //design parameter
  lambda_ = 3 - n_x_;
  
  //sigma points count
  n_sig_ = 2 * n_aug_ + 1;
  
  //weights
  weights_ = VectorXd(n_sig_);
  //set weights
  
  double weights0 = lambda_ / (lambda_ + n_aug_);
  double weightsn = 1. / (2. * (lambda_ + n_aug_));
  
  for(int i = 0; i < n_sig_; ++i){
    weights_(i) =  i == 0 ? weights0 : weightsn;
  }

  n_z_radar_ = 3;  
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  R_radar_ << pow(std_radr_, 2.), 0., 0.,
              0., pow(std_radphi_, 2.), 0.,
              0., 0., pow(std_radrd_, 2.);
              
  n_z_lidar_ = 2;
  R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
  R_lidar_ << pow(std_laspx_, 2.), 0.,
              0., pow(std_laspy_, 2.);
              
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);
  

  NIS_lidar_ = 0.;
  NIS_radar_ = 0.;
}

UKF::~UKF() {}

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
  //cout<<"---------UKF::ProcessMeasurement-----------"<<endl;

  if((use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) || 
     (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)){
    
    if(is_initialized_){
      double delta_t = (meas_package.timestamp_ - time_us_)/1000000.;
      Prediction(delta_t);
      time_us_ = meas_package.timestamp_;
    }
    else{         
      is_initialized_ = true;
      time_us_ = meas_package.timestamp_;
     
      if(use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR){     
        double rho = meas_package.raw_measurements_(RHO);
        double phi = meas_package.raw_measurements_(PHI);
        double rho_dot = meas_package.raw_measurements_(RHODOT);
        double x = rho * cos(phi);
        double y = rho * sin(phi);
        double vx = rho_dot * cos(phi);
        double vy = rho_dot * sin(phi);
        double v = sqrt(vx * vx + vy * vy);
        x_ << x, y, v, 0., 0.;
        return;      
      }
      else if(use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER){
        double x = meas_package.raw_measurements_(PX);
        double y = meas_package.raw_measurements_(PY);
        x_ << x, y, 0., 0., 0.;
        return;      
      }      
    }


    if(use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR){      
      UpdateRadar(meas_package);    
    }
    else if(use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER){      
      UpdateLidar(meas_package);    
    }
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
  //cout<<"---------UKF::Prediction-----------"<<delta_t<<endl;
  
   //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
  
  //create augmented mean state  
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_ + 1) = 0;
  
  //create augmented covariance matrix
  P_aug.fill(0.);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_aug_-2, n_aug_-2) = pow(std_a_, 2.);
  P_aug(n_aug_-1, n_aug_-1) = pow(std_yawdd_,2.);
    
  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_; ++i){
      Xsig_aug.col(i + 1) = A.col(i) * sqrt(lambda_ + n_aug_) + x_aug;
      Xsig_aug.col(i + 1 + n_aug_) = -A.col(i) * sqrt(lambda_ + n_aug_) + x_aug;
  }
 
  //predict sigma points
  for(int i = 0; i < n_sig_; ++i){
      
    double v = Xsig_aug(V, i);
    double psi = Xsig_aug(PSI, i);
    double psi_dot = Xsig_aug(PSIDOT, i);
    double nu_a = Xsig_aug(NUA, i);
    double nu_psi = Xsig_aug(NUPSI, i);
      
    bool psi_dot_zero = fabs(psi_dot) < 1e-10;
      
    Xsig_pred_(PX, i) = Xsig_aug(PX, i) + (!psi_dot_zero ? 
      (v / psi_dot * (sin(psi + psi_dot * delta_t) - sin(psi))) : 
      (v * cos(psi) * delta_t)) + 1./2.*pow(delta_t, 2.) * cos(psi) * nu_a;
    
    Xsig_pred_(PY, i) = Xsig_aug(PY, i) + (!psi_dot_zero ? 
      (v / psi_dot * (-cos(psi + psi_dot * delta_t) + cos(psi))) : 
      (v * cos(psi) * delta_t)) + 1./2. * pow(delta_t, 2.) * sin(psi) * nu_a;
        
    Xsig_pred_(V, i) = Xsig_aug(V, i) + delta_t * nu_a;    
    Xsig_pred_(PSI, i) = Xsig_aug(PSI, i) + psi_dot * delta_t  + 1./2.*pow(delta_t, 2.) * nu_psi;
    Xsig_pred_(PSIDOT, i) = Xsig_aug(PSIDOT, i)  + delta_t * nu_psi;              
  }

  //predict state x
  x_.fill(0.);
  for(int i = 0; i < n_sig_; ++i)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predict covariance P
  P_.fill(0.);
  for(int i = 0; i < n_sig_; ++i)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = tools_.Truncate2Pi(x_diff(3));
    P_ += weights_(i) * x_diff * x_diff.transpose();  
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

  MatrixXd Zsig = MatrixXd(n_z_lidar_, n_sig_);
  for(int i = 0; i < n_z_lidar_; ++i){
    Zsig.row(i) = Xsig_pred_.row(i);
  }
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_lidar_);
  z_pred.fill(0.0);
  for(int i = 0; i < n_sig_; i++){
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_lidar_, n_z_lidar_);
  S.fill(0.0);
  for(int i = 0; i < n_sig_; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S += R_lidar_;

  //lidar measurement
  VectorXd z = meas_package.raw_measurements_;

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);

  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; ++i){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K
  MatrixXd K = Tc * S.inverse();

  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  //Normalized innovation square for consistency check
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
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

  //cout<<"---------UKF::UpdateRadar-----------"<<endl;
  
   //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, n_sig_);

  //mean predicted measurement
  MatrixXd z_pred = VectorXd(n_z_radar_);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);

  //transform sigma points into measurement space
  for(int i = 0; i < n_sig_; ++i){
    double px = Xsig_pred_(PX, i);
    double py = Xsig_pred_(PY, i);
    double v = Xsig_pred_(V, i);
    double psi = Xsig_pred_(PSI, i);
      
    double rho = sqrt(pow(px, 2.) + pow(py, 2.));
    double phi = atan2(py,px);
    double rho_dot = (px*cos(psi)*v + py*sin(psi)*v)/rho;
      
    Zsig(RHO, i) = rho;
    Zsig(PHI, i) = phi;
    Zsig(RHODOT, i) = rho_dot;      
  }
  
  //calculate mean predicted measurement  
  z_pred.fill(0.);
  for(int i = 0; i < n_sig_; ++i){
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  S.fill(0.);  
  for(int i = 0; i < n_sig_; ++i){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(PHI) = tools_.Truncate2Pi(z_diff(PHI));
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R_radar_;
  
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
  VectorXd z = meas_package.raw_measurements_;
  Tc.fill(0.);
  for(int i = 0; i < n_sig_; ++i){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    x_diff(PSI) = tools_.Truncate2Pi(x_diff(PSI));
    z_diff(PHI) = tools_.Truncate2Pi(z_diff(PHI));
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
  
  //calculate Kalman gain K
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;
  z_diff(PHI) = tools_.Truncate2Pi(z_diff(PHI));
  
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();
    
  //Normalized innovation square for consistency check
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;  
 
}

const VectorXd& UKF::StateVector() const{
  return x_;
}

double UKF::NisRadar() const{
  return NIS_radar_;
}
  
double UKF::NisLidar() const{
  return NIS_lidar_;
}
  
