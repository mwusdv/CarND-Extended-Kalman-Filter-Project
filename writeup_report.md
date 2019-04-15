# **Extended Kalman Filter Project** 


## Basic Description

* The tracking logic, where EKF is called, is contained the FusionEKF.cpp
* The update/correction steps corresponding to the laser and radar measurements are `KalmanFilter::laserUpdate` and `KalmanFilter::radarUpdate` respectively.
* For `KalmanFilter::radarUpdate`:
    * In addition to the radar measurement, the current Jacobian martrix is also passed into the function.
    * As the first step, the difference value `y` between the measurement and the one derived from the current state is calculated. `y(1)` is the orientation angle. I make it within the range of [-pi, pi] by keep adding `2*pi` or substracting `2*pi` from it. This really affects the final RMSE.


## Resuts

My code can get RMSE for px, py, vx, vy less than [.11, .11, 0.52, 0.52] on both data1 and data2. Here are the screen shots:

Result on data1: 
![alt text][image1]

Result on data2: 
![alt text][image2]

[//]: # (Image References)
[image1]: ./data1.jpg
[image2]: ./data2.jpg


## Some Thoughts about Code Design
In the current design, there is no clear boundary between the `FusionEKF` class and the `KalmanFilter` class. The `FusionEKF` directly initizalies and modifes the matrices of the `KalmanFilter` class. This is fine if we think `FusionEKF` has the full information of the whole process: the motion model, the measurment noise, etc. In this case the `KalmanFilter` class simply executes the (extended) Kalman filter algotihm without knowing anything about the process or object it is tracking. However, in the radar update step, when we calculate the difference value `y`, `KalmanFilter` class still needs the specific information of the proecss, like polar coordinates, etc. 

So another possible design may be to put everything into the `KalmanFilter` class, including initialization, matrix modification, Jacobian calculation, etc. This way, `FusionEKF` will not directly access the parameters of the `KalmanFilter` class, it just passes the timestamp and measuments into the `KalmanFilter` and let the `KalmanFilter` to process everything.