#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>     //DWB added math library for using sin/cos 

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

  // ekf_.R_laser_ = MatrixXd(2, 2);
  // ekf_.R_radar_ = MatrixXd(3, 3);
  // ekf_.H_laser_ = MatrixXd(2, 4);
  // ekf_.Hj_ = MatrixXd(3, 4);

  //maybe don't want to put ekf_ in front of these ... at some later point I'll say
  // ekf_.R = R_laser??
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);                 //FIXME, not sure if I tied this is correctly


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
  
  // measurement matrix (For LiDAR, no velocity measured but it is part of state, H helps transform this)
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  
  //Setting Q the process noise - modeling acceleration (change in velocity) as noise since not tracked by my states
  ekf_.Q_ = MatrixXd(4, 4);


  //why not set F and P here since these shouldn't change
  ekf_.F_ = MatrixXd(4, 4);
    
  //Initial F matrix - doesn't incorporate delta_t yet
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  
  // FIXME where did these values of P come from? - think about this one. 1000!!!
  //ekf_.P_ << 1, 0, 0, 0,
  //          0, 1, 0, 0,
  //          0, 0, 1000, 0,
  //          0, 0, 0, 1000;

  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 100, 0,   //note that 100 got close to rubric
            0, 0, 0, 100;   // 100

  
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
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.   //done in constructor step above, created P
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "Starting EKF... " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float temp_px = 0;
      float temp_py= 0;
      float temp_ro = measurement_pack.raw_measurements_[0];
      float temp_theta = measurement_pack.raw_measurements_[1];

      cout<<"temp_ro "<<temp_ro<<endl;
      cout<<"temp_theta "<<temp_theta<<endl;
      

      //not needed - measurement is always in correct range
      // if(temp_theta>M_PI) {

      //   temp_theta = temp_theta - (2*M_PI);
      //   cout << "DWB Adjusting temp_theta minus 2pi " << endl;
      // }

      // if(temp_theta<M_PI) {

      //   temp_theta = temp_theta + (2*M_PI);
      //   cout << "DWB Adjusting temp_theta plus 2pi " << endl;
      // }

      temp_px = temp_ro*(cos(temp_theta));  //swapping does not have big effect
      temp_py = temp_ro*(sin(temp_theta));

      cout<<"temp_px "<<temp_px<<endl;
      cout<<"temp_py "<<temp_py<<endl;
      
      ekf_.x_ << temp_px, 
              temp_py, 
              0, 
              0;


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
          // set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
    previous_timestamp_ = measurement_pack.timestamp_;
    //ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, ekf_.H_, ekf_.R_, ekf_.Q_);  //FIXME - not using the ekf.Init() function right now - doesn't seem like I need it!??
  }

  /**
   * Prediction Setp (Step 1of2)
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // DWB Pasted in almost directly from the course lecture
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  //float dt = (measurement_pack.timestamp_ - previous_timestamp_);
  previous_timestamp_ = measurement_pack.timestamp_;

  if (dt==0){
    cout<<"dt is zero, may need to handle this better";
  }
  
    // From Course Quiz - Your Code Here Comment
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q

  float noise_ax = 9;   //using suggested values here
  float noise_ay = 9;   

  //ekf_.Q_ = MatrixXd(4, 4);  //defined as 4x4 in constructor
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;



  ekf_.Predict();

  /**
   * Update Step (Step 2of2)
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    
    ekf_.H_ = Hj_;  /// I suppose this will have to be a function call to tools ....FIXME

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);  //call the updateEKF function since radar

  } else {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);   //call the update function since laser

  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;  //FIXME - commented this out during Tshoot
  //cout << "P_ = " << ekf_.P_ << endl;
}
