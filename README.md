# Unscented Kalman Filter Project

This project is an implementation of Unscented Kalman Filter. It is used to estimate the state of a moving object of interest with noisy lidar and radar measurements.
Correct implementation should pass the following criteria: px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30]. 

UKF

* allows to take noisy measurement data as input and provide a smooth position and velocity estimation of dynamic objects without introducing a delay
* provides estimation of the orientation and yaw rate of dynamic objects using sensors that does not directly provide this information
* provides information of how precise the estimation

## Results

[image1]: ./images/output_video.gif
[image2]: ./images/nis_radar.png
[image3]: ./images/nis_lidar.png

Example of the filter performance on the simulator

![alt text][image1]

Final RMSE approximately [0.07 0.08 0.35 0.24]. Consistency check for the filter is based on Normalized Innovation Squared metric. Good results achieved with the following noise values:
process noise standard deviation longitudinal acceleration 2 m/s^2, process noise standard deviation yaw acceleration PI/8 rad/s^2.

![alt text][image2]

![alt text][image3]

## Data input and output

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Dependencies
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 
* [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

