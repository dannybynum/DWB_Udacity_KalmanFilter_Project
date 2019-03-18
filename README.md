[//]: # (Image References)

[image1]: ./Simulator_Output.PNG "Screenshot of Simulator Receiving State Estimates from ExtendedKF program"

# Danny's Extended Kalman Filter Project 

This repository contains all the source files and compiler instructions needed in order to build an executable called `ExtendedKF` which can provide a kalman-filtered estimate of an object state measured by LiDAR and RADAR sensors.  The object state is composed of 2-D position and 2-D velocity.  The `ExtendedKF` executable expects to interact with a car/object-data simulator built by Udacity (which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)).  This simulator provides the LiDAR and RADAR measurements and the `ExtendedKF` executable takes in these measurements, performs the Kalman Filter operations and then provides the measurements back to the simulator in order to be plotted.  The program also calculates the Root-Mean-Squared-Error (RMSE) for the object state estimates and the ground truth measurements (RADAR and LiDAR).  

The training performance achieved with this last model (Version 7) and assoicated training data (20% validation set split off after shuffle):
![alt text][image1]



##Udacity Provided Starter Code and Resources
The following files were all provided in their initial form from the [project repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).   The files in **bold** is what I modified to implement the Kalman Filter and make the program work as intended.
* src/main.cpp
* src/**FusionEKF.cpp**
* src/FusionEKF.h 
* src/**kalman_filter.cpp**
* src/kalman_filter.h
* src/**tools.cpp**
* src/tools.h

As previously mentioned the simulator program can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

I used a local Windows 10 environment setup in order to code and compile this program.  It was seemless/easy to setup by installing and using an "app" called Ubuntu Bash 16.04.  Here are the steps I took:
* Setup Ubuntu Bash using this [guide](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)

Enter the following commands in the terminal:
* `git clone https://github.com/dannybynum/DWB_Udacity_KalmanFilter_Project`
* `./install-ubuntu.sh`  //this installed all of the compilers as well as the uWebSocketIO utility that is needed for `ExtendedKF` "communicate" with the simulator
* `mkdir build && cd build` then `cmake .. && make` then `./ExtendedKF`  //this should test the building/compiling and then running of the program

Copying files back and forth from my windows environment where I used the text editor
```
sudo cp -a /mnt/c/users/bynum/documents/udacity/term1/project5_ekf_windows/src/. src/
```


##Brief Code Walk-through

main.cpp (Udacity provided code):
---
Read in the measurement data - example shown for LASER data:
```cpp
if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
```

Call the main function - note this gets called for every measurement
```cpp
// Call ProcessMeasurement(meas_package) for Kalman filter
          fusionEKF.ProcessMeasurement(meas_package);       
```

Send the estimations and ground truth over to the function that calculates the RMSE
```cpp
VectorXd estimate(4);

          double p_x = fusionEKF.ekf_.x_(0);
          double p_y = fusionEKF.ekf_.x_(1);
          double v1  = fusionEKF.ekf_.x_(2);
          double v2 = fusionEKF.ekf_.x_(3);

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;
        
          estimations.push_back(estimate);

          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
```

FusionEKF.cpp (my code):
---

Initialize the State Vector x - Use the first measurement - example shown for Laser:
```cpp
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
```
[...]
```cpp
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
          // set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    }
```


Figure out how much time has passed and use this to predict the new state based on previous state and elapsed time (Kalman Process Step1):
```cpp
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
```
[...]
```cpp
 // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
```
[...]
```cpp
ekf_.Predict();
```

Now that we have a predicted state lets combine this with the new measurement and then publish the Kalman State Estimate (Kalman Process Step2):
(Note the example shown is for Laser)
```cpp
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);   //call the update function since laser
```


kalman_filter.cpp (my code):
---
The Predict() and Update() steps (called in FusionEKF.cpp) are defined in the `kalman_filter.cpp` file as follows:
(Note the update example is for Laser)

```cpp
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
```
[...]
```cpp
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
```