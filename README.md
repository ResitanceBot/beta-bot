# BETA BOT PROJECT
![Quadrotor_image](https://static.wikia.nocookie.net/doblaje/images/7/7b/Donald.jpg/revision/latest?cb=20220512112816&path-prefix=es)

## Project description

## Installlation
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
This is intended as a deliverable for a university course. Terefore, probably will not receive any updated from now on. However, feel free to fork it and expand its functionalities ;)
