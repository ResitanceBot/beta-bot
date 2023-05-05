#!/bin/bash

# Eliminación del warning "Invalid argument passed to canTransform argument source_frame in tf2 frame_ids cannot be empty"
# Descartar stderr de rviz: el warning producido se debe a topics internos de rviz (/move_base_simple/goal e /initialpose)
# que nosotros no llegamos a usar
rosrun rviz rviz -d `rospack find beta_bot_bringup`/rviz_cfg/outdoor_flight.rviz 2>/dev/null

