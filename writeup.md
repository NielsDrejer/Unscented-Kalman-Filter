## Udacity Self-Driving Car Engineer Nanodegree, Term 2, Project 2: Unscented Kalman Filter

---

The goal of this project is to implement an unscented Kalman Filter in C++, which fusions Lidar and Radar Measurements to track a vehicle. Noisy Radar and Lidar measurements are provided, as well as a Simulator App which can visualize the measurements and the results of the Extended Kalman Filter calculations. Seed code for the project is provided here:

https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project.git

[//]: # (Image References)
[image1]: ./writeup_images/dataset1.png
[image2]: ./writeup_images/NISLaser.png
[image3]: ./writeup_images/NISRadar.png
[image4]: ./writeup_images/radaronly.png
[image5]: ./writeup_images/laseronly.png

## Notes about Installation

I am using a Mac for development.

The uWebSockets were already installed in connection with project 1. So I could directly execute the instructions:

1. mkdir build
2. cd build
3. cmake ..
4. make

As in project 1 I had to modify the generated CMakeLists.txt and instead use the following line to enable make to find the linker:

link_directories(/usr/local/Cellar/libuv/1.20.3/lib)

The original line referred to the folder /usr/local/Cellar/libuv/1*/lib, and that does not work on my system.

After that everything worked as expected. I used the Sublime text editor and make on the command line to build.

## Notes about Implementation

I used the provided file structure and basically filled in the missing bits:

1. Completed tools.cpp, including the function that calculate root mean squared error (RMSE).

2. Completed ukf.cpp, including completing the init function and the functions ProcessMeasurement(), Prediction(), UpdateLidar() and UpdateRadar().

In most cases I used the code I had developed in the lesson exercises. Very important is to normalize any angle as soon as an operation means it could be outside of the -pi to +pi range, so I made a small function for that.

I also added code to store the NIS values for radar and laser in a file called nis.txt.

Finally I include a simple jupyter notebook which I made for plotting the NIS diagrams.

## Rubric Points

---

#### 1. Compiling  

The code compiles and links when calling 'make all' on the command line.

#### 2. Accuracy

Your px, py, vx, and vy RMSE should be less than or equal to the values [.9, .10, 0.40, 0.30].

When adjusting the process noise standard deviation longitudinal acceleration stda to 1.0, and the process noise standard deviation yaw acceleration stdyawdd to 0.5, I produced the result seen here:

![alt text][image1]

#### 3. Follows the correct Algorithm

My code follows the algorithms from the lessons. The code in ukf.cpp should be easy to verify as I used the same variable names as in the lesson material, and because I just reused the code structure from the seed project.

It is worthwhile here to explain the implementation of the UpdateLidar() function, as this was not examplified in the lessons. The function is found in line 281-354 in ukf.cpp. The main thing that differentiate it from the UpdateRadar() function is that there is no transformation of sigma points into measurement space. I simply use the px and py values of the sigma points directly. The rest is the same as in the lessons and in UpdateRadar().

#### 4. First Measurements

In ProcessMeasurement() line 115-158 you can see how I initialize the state vector based on the first measurement. In case the first measurement is a RADAR measurement it is possible to calculate an initial value of the velocity v. This is not possible for LASER measurements. In both cases we can initialize the x and y position in the state vector, and in both cases I set the yaw and yaw rate to 0.

The initialization of the state covariance matrix P is done in the init function line 79-84. P is initialized to the identity matrix as adviced in the lessons.

The laser and radar measurement noise covariance matrices are also initialized in the init function, in lines 86-97.

#### 5. Predict then Update

For all measurements following the first measurement, the code in line 160-178 is used. As can be clearly seen here first the prediction step is executed and then the update step. Different update functions are called for RADAR and LASER measurements.

#### 6. Handle Radar and Lidar measurements

The code handles these measurement types differently in the functions UpdateRadar() and UpdateLidar().

#### 7. Code Efficiency

In general I actually programmed for code clarity rather than compactness. There are clearly parts of the code that could be written in a more efficient way. For example some steps in the UpdateRadar() and UpdateLidar() are repeated (calculation of innovation covariance matrix S and cross correlation matrix Tc both involve calculating the measurement space difference z_diff = Zsig.col(i) - z_pred for all sigma points).

#### 8. Standout

I calculate the NIS values for Radar and Laser measurements in line 352 and 465, and write the values to a file called nis.txt. Using a jupyter notebook I print the values. I played around with the process noise parameters stda and stdyawdd. The values 1.0 and 0.5 produce the following NIS diagrams:

![alt text][image2]

![alt text][image3]

In both cases I also plotted the 95% line as given in the course material for 2 dimensions (5.991) and 3 dimensions (7.815). Both diagrams look reasonable and suggest the unscented kalman filter I implemented is sound.

It shall be noted that the first value for radar NIS is so much out of range that I did not plot it. I assume this has to do with which measurement comes first in the measurement file, but I am not sure about it.

### Conclusion

I believe all rubric points have been fulfilled.

Comparing to the results I achived with the enhanced kalman filter in project 1 clearly the unscented kalman filter works better. The enhanced filter produced RMSE values of X: 0.0973, Y: 0.0855, VX: 0.4513 and VX: 0.4399. The unsented filter of the current project produced X: 0.0661, Y: 0.0827, VX: 0.3323 and VY: 0.2145, meaning all RMSE values are significantly better.

I also ran my filter using radar measurement only and got the following results:

![alt text][image4]

When using laser measurements only the following results were produced:

![alt text][image5]

Both cases are worse than the combined scenario, which is to be expected. Radar measurements only work better than laser measurements only for the velocity RMSE values VX and VY. For the position RMSE values X and Y it is the other way around, laser measurements only produces the best results.
