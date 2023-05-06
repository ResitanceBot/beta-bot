#!/bin/bash

# Lanza el robot_localization sin que salgan warnings relativos a que puntualmente se toma
# una muestra antigua para la localización
roslaunch beta_bot_localization robot_localization.launch > /dev/null 2>&1