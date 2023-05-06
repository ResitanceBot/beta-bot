#!/bin/bash


WORLD_NAME="$1.world"
echo "[worldLaunch.sh] WORLD_NAME = $WORLD_NAME"
# Lanza el mundo sin que salgan los warning de "a Gazebo le falta X textura"
 roslaunch gazebo_ros empty_world.launch \
    world_name:="$(find beta_bot_gazebo)/worlds/$WORLD_NAME" \
    use_sim_time:=true \
    gui:=true \
    paused:=false