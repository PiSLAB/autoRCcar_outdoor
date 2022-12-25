#!/bin/bash

gnome-terminal -- ros2 run autorccar_ubloxf9r ubloxf9r
sleep 1

gnome-terminal -- ros2 run autorccar_navigation gnss_ins
sleep 1

gnome-terminal -- ros2 run autorccar_path_planning bezier_curve
sleep 1

gnome-terminal -- ros2 run autorccar_control pure_pursuit
