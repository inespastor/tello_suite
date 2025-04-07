# Tello Workspace Guide

1. **Run this Repository**
Take into account that the drone works on time and velocity, so the distances and angles are approximate. 

valid_commands = {'takeoff', 'land', 'forward', 'backward', 'right', 'left', 'up', 'down', 'rotate_r', 'rotate_l'}

takeoff & land

All the commands with movement are executed on meters

All the commands of direction are executed with angles. 

#TO EXECUTE:
# ros2 launch tello_driver tello_driver.launch.py
# ros2 run text_control_plugin text_control_plugin --ros-args -p standalone:=true
After you need to takeoff the drone, next you can put any command, finally you will need to land the drone. 


