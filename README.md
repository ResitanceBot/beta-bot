# BETA BOT PROJECT
![Quadrotor_image](https://static.wikia.nocookie.net/doblaje/images/7/7b/Donald.jpg/revision/latest?cb=20220512112816&path-prefix=es)

## Project description
This project consists of the EKF-based localization of a quadrotor aerial robot. The approach we propose focuses on urban air mobility, where having accurate vehicle localization is of paramount importance for enabling increasingly autonomous and secure operations of these vehicles in cities.

[DRONE PHOTOS]

This model of "air taxi" incorporates the following sensors (implemented through Gazebo and custom plugins):

- Stereo Camera
- GNSS
- IMU
- Magnetometer
- Beacon-based localization signal receivers

In this work, the possibilities of the Extended Kalman Filter have been explored, a filter where the inclusion of sensors of different nature poses a series of challenges that can be overcome thanks to the flexibility of this estimator.

The working philosophy to be followed will consist of progressively including the different sensors, comparing and analyzing their contributions to the robot's pose estimation results. For this purpose, different phases are proposed:

- EKF v1: EKFPrediction: IMU Accelerometer and Gyroscope; EKFUpdate: IMU Accelerometer, Magnetometer and GNSS
- EKF v2: EKFPrediction: IMU Accelerometer and Gyroscope; EKFUpdate: IMU Accelerometer, Magnetometer and Beacons
- EKF v3: EKFPrediction: IMU Accelerometer and Gyroscope; EKFUpdate: IMU Accelerometer, Magnetometer, GNSS and Beacons
- EKF v4: EKFPrediction: Visual Odometry; EKFUpdate: IMU Accelerometer, Magnetometer, GNSS and Beacons
- EKF v5: EKFPrediction: Visual Odometry (position), IMU Gyroscope: Orientation; EKFUpdate: IMU Accelerometer, Magnetometer, GNSS and Beacons

An initialization node has been developed, based on the Gauss-Newton algorithm, common to all versions, which will be used to initialize the state vector in the filter.

## Installation
### From Ubuntu
- Prerequisites: Ubuntu 20.04 LTS - ROS Noetic
```
cd ~ # IT IS IMPORTANT TO CLONE REPO IN 
git clone https://github.com/ResitanceBot/beta-bot.git
cd beta-bot
rosdep install --from-paths src --ignore-src –r –y
catkin_make
source devel/setup.sh
roslaunch beta_bot_bringup start_simulation.launch
```
### Docker (plug&play)
(in develop)

### Configuration
 You can complete main launch of simulation with different params (```roslaunch beta_bot_bringup start_simulation <param> ```:
 * teleop_mode_selection=window[default]/keypad
E.g. ```roslaunch beta_bot_bringup start_simulation teleop_mode_selection=keypad``` in order to use PS3/PS4/Xbox controller to control drone movements.

## Development
This is intended as a deliverable for a university course. Therefore, probably will not receive any updated from now on. However, feel free to fork it and expand its functionalities ;)
