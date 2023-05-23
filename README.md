# BETA BOT PROJECT
![Quadrotor_image](https://github.com/ResitanceBot/beta-bot/blob/doc/images/Presentation.png)

## Project description
This project consists of the EKF-based localization of a quadrotor aerial robot. The approach we propose focuses on urban air mobility, where having accurate vehicle localization is of paramount importance for enabling increasingly autonomous and secure operations of these vehicles in cities.

We have implemented this functionality as a ROS Noetic package, running in Ubuntu 20.04 LTS. 

![Middle_image](https://github.com/ResitanceBot/beta-bot/blob/doc/images/Middle.png)

This model of "air taxi" incorporates the following sensors (implemented through Gazebo and custom plugins):

- Stereo Camera
- GNSS
- IMU
- Magnetometer
- Beacon-based localization signal receivers

![Beacons_image](https://github.com/ResitanceBot/beta-bot/blob/doc/images/Beacons.png)

In this work, the possibilities of the Extended Kalman Filter have been explored, a filter where the inclusion of sensors of different nature poses a series of challenges that can be overcome thanks to the flexibility of this estimator.

The working philosophy to be followed will consist of progressively including the different sensors, comparing and analyzing their contributions to the robot's pose estimation results. For this purpose, different phases are proposed:

- EKF v1: EKFPrediction: IMU Accelerometer and Gyroscope; EKFUpdate: IMU Accelerometer, Magnetometer and GNSS
- EKF v2: EKFPrediction: IMU Accelerometer and Gyroscope; EKFUpdate: IMU Accelerometer, Magnetometer and Beacons
- EKF v3: EKFPrediction: IMU Accelerometer and Gyroscope; EKFUpdate: IMU Accelerometer, Magnetometer, GNSS and Beacons
- EKF v4: EKFPrediction: Visual Odometry; EKFUpdate: IMU Accelerometer, Magnetometer, GNSS and Beacons
- EKF v5: EKFPrediction: Visual Odometry (position), IMU Gyroscope: Orientation; EKFUpdate: IMU Accelerometer, Magnetometer, GNSS and Beacons

![Results_image](https://github.com/ResitanceBot/beta-bot/blob/doc/images/Results.png)

An initialization node has been developed, based on the Gauss-Newton algorithm, common to all versions, which will be used to initialize the state vector in the filter.

![GNini_image](https://github.com/ResitanceBot/beta-bot/blob/doc/images/GNini.png)

## Tested Platforms
This ROS package has been tested in the following environments:
- Native Ubuntu 20.04 LTS - ROS 1 Noetic
- VirtualBox and VMware virtual machines 
- WSL2
- Docker image

As a recommendation, this ROS package requires a powerful GPU to run smoothly. On some occasions, it may take several minutes for the simulation to fully load.

## Installation
### From Ubuntu
This installation method will also work for WSL2 and virtual machines.

- Prerequisites: Ubuntu 20.04 LTS - ROS Noetic
- If selected: WSL2 or virtual machine installed and running Ubuntu 20.04 LTS - ROS Noetic
```
cd ~ # IT IS IMPORTANT TO CLONE REPO IN 
git clone https://github.com/ResitanceBot/beta-bot.git
cd beta-bot
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.sh
roslaunch beta_bot_bringup start_simulation.launch
```
### Docker (plug&play)
If you have Docker installed, we have developed a Docker container based on the "nvidia/cuda" container, which you can build using the Dockerfile provided in the doc branch "/custom_docker_images/nvidia_ros/Dockerfile". First, you will need to have the nvidia drivers of your GPU correctly installed. 

Next, you have to install "nvidia-container-toolkit". You can do it following these steps:
```
sudo apt-get install curl       
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo apt-get install -y nvidia-container-toolkit-base

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
And now, we can test the Docker container "nvidia/cuda:11.6.2-base-ubuntu20.04":

```
sudo docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi

```
If the test runned correctly, you now have this NVIDIA container available among your Docker images. And now, you can use our Dockerfile to build the Docker container:

```
cd ~
git clone -b doc https://github.com/ResitanceBot/beta-bot.git
cd beta-bot/custom_docker_images/nvidia_ros
sudo docker build -t nvidia_ros .      
```
And now, with the Docker container built, you can start it with this command (this command requires a PS3/PS4/Xbox controller conected at port /dev/input/js0. If you don't have a controller, delete the line "    --device="/dev/input/js0" \"):

```
sudo docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device="/dev/input/js0" \
    nvidia_ros \
    bash
```

This container includes Ubuntu 20.04 and ROS Noetic installed, as well as the necessary dependencies to make your PC's GPU work within the container. It also has the repository installed and compiled, so you can launch it by simply running the following commands (inside the container):

```
cd beta-bot
source devel/setup.sh
roslaunch beta_bot_bringup start_simulation.launch 
```

The first time you launch this package, it may take a couple of minutes to start. On subsequent occasions, it should not take more than a minute.

## How to launch this simulation
This project has an unique main launch, which can be called with these commands after installing the package:

```
cd ~/beta-bot
source devel/setup.sh
roslaunch beta_bot_bringup start_simulation.launch
```

You can complete this main launch of the simulation with different params (```roslaunch beta_bot_bringup start_simulation.launch <param>```):

 * teleop_mode_selection:=window[default]/keypad
E.g. ```roslaunch beta_bot_bringup start_simulation.launch teleop_mode_selection:=keypad``` in order to use PS3/PS4/Xbox controller to control drone movements.

 * version:=v1/v2/v3/v4[default]/v5
E.g. ```roslaunch beta_bot_bringup start_simulation.launch version:=v5``` for selecting the implemented EKF version (all versions explained at the "Project Description") that you want to launch.

 * use_beta_bot_localization:=true[default]/false
E.g. ```roslaunch beta_bot_bringup start_simulation.launch use_beta_bot_localization:=false```. If true, the robot will use our custom EKF. If false, the robot will use the ROS package "robot_localization" to perform its localization.

 * world_name:=small_city[default]/empty/cyberzoo
E.g. ```roslaunch beta_bot_bringup start_simulation.launch world_name:=empty``` in order to choose one of these three worlds. The default world is the one who takes at least 45 seconds to launch ;)
For this reason, we added a simplified empty world, called "empty", with nothing but the robot and the beacons, to test the first three versions of the EKF (you cannot test visual odometry from v4 and v5 in an empty world).

In fact, you can also use more than one of these parameters to perform the simulation as you want. For example, you can run ```roslaunch beta_bot_bringup start_simulation.launch version:=v3 teleop_mode_selection:=keypad world_name:=empty``` for running EKF_v3 in an empty world in Gazebo, and you will use the PS3/PS4/Xbox controller to control drone movements.  

## Development
This is intended as a deliverable for a university course. Therefore, probably will not receive any updated from now on. However, feel free to fork it and expand its functionalities ;)

