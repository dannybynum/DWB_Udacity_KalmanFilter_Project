#include "kalman_filter.h"
#include <math.h>     //DWB added math library for using sin/cos 
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
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





void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //VectorXd z_pred_rad = H_ * x_;  //update this, manual calculation I suppose...
  //Starting here this is all to come up with polar conversion for state prediction
  float temp_px = x_(0);
  float temp_py = x_(1);
  float temp_vx = x_(2);
  float temp_vy = x_(3);

  //Monday Mar-18 Change, add this back in with values as suggested by project review
  // check division by zero
  if (fabs(temp_px) < 0.001) {
    cout<<temp_px<<" ";
    cout << "DWB adjusting temp_px" << endl;
    temp_px = 0.001;  
  }

  if (fabs(temp_py) < 0.001) {
    cout<<temp_py<<" ";
    cout << "DWB adjusting temp_py" << endl;
    temp_py = 0.001;  
  }

  // Similar to code from lecture, pre-compute a set of terms to avoid repeated calculation
  float c1 = sqrt((temp_px*temp_px)+(temp_py*temp_py));
    
  if (fabs(c1) < 0.001) {
    cout<<c1<<" ";
    cout << "DWB adjusting c1" << endl;
    c1 = 0.001;  
  }


  float c2 = atan2(temp_py,temp_px);   //using atan2 because it returns -pi to pi

  
 
  float c3 = ((temp_px*temp_vx)+(temp_py*temp_vy))/c1; //FIXME - check divide by zero here


// Not needed - atan2 returns in correct range -pi to +pi
  // if(c2>M_PI) {

  //   c2 = c2 - (2*M_PI);
  //   cout << "DWB Adjusting c1 minus 2pi " << endl;
  // }

  // if(c2<-M_PI) {

  //   c2 = c2 + (2*M_PI);
  //   cout << "DWB Adjusting c1 plus 2pi " << endl;
  // }


  // Finally - now I can manually create the predicted measurement in polar coordinates
  // this is used for calculating the error  
  VectorXd z_pred_rad = VectorXd(3);

  z_pred_rad<< c1, 
               c2,
               c3;
                          

  VectorXd y_rad = z - z_pred_rad;          //renamed y_rad


  if(y_rad[1]>M_PI) {
    y_rad[1] = y_rad[1] - (2*M_PI);
    cout << "DWB Adjusting y_rad[1] minus 2pi " << endl;
  }

  if(y_rad[1]<-M_PI) {
    y_rad[1] = y_rad[1]+ (2*M_PI); 
    cout << "DWB Adjusting y_rad[1] plus 2pi " << endl;
  }



  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y_rad);                    //using y_rad radar here
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
