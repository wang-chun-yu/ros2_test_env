# !/bin/bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/home/chunyu/work/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False