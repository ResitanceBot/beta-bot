import numpy as np


def gen_launch_beacon(beacon_name: str, x: float, y: float, z: float) -> str:
    a = f'''\n<include file="$(find beacon_gazebo_plugin)/launch/beacon.launch" >
                <arg name="init_pose" value="-x {x} -y {y} -z {z}" />
                <arg name="beacon_name" value="{beacon_name}" />
            </include>\n'''
    return a


def gen_launch_robot(robot_name: str, x: float, y: float, z: float, yaw: float = 0) -> str:
    b = f'''\n<include file="$(find beacon_gazebo_plugin)/launch/robot.launch" >
                <arg name="init_pose" value="-x {x} -y {y} -z {z} -yaw {yaw}" />
                <arg name="robot_name" value="{robot_name}" />
              </include>\n'''
    return b




