#!/bin/bash

# 设置库搜索路径
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Source ROS2 环境（如果需要）
source /opt/ros/humble/setup.bash

# 构建项目
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug