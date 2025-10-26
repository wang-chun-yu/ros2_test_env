[TOC]
# build docker images
- create ros images
cd {your_workspace}/ros2_test_env/docker/
docker build --network host -f base.dockerfile . -t ros_humble:latest
- add user

# run docker container
./docker/scripts/normal_env_run.sh

# ~~build navigation2~~
```
export ROS_DISTRO=humble
cd ~/work/ros2_test_env/src
git clone https://github.com/ros-navigation/navigation2.git --branch main
docker build --tag navigation2:$ROS_DISTRO   --build-arg FROM_IMAGE=ros:$ROS_DISTRO   --build-arg OVERLAY_MIXINS="release ccache lld"   --cache-from ghcr.io/ros-navigation/navigation2:main   ./navigation2 --network host --no-cache 
```
# download navigation2 && turtlebot3 src
git clone https://github.com/ros-navigation/navigation2.git --branch humble
git clone git@github.com:ROBOTIS-GIT/turtlebot3_simulations.git --branch humble
# create docker container
./docker/scripts/normal_env_run.sh
# into docker container
./docker/scripts/normal_env_into.sh
# add navigation2 dependences
sudo apt update
sudo apt install python3-rosdep
rosdep init
rosdep update
cd ~/work
rosdep install --from-paths src/navigation2 --ignore-src -r -y
# source env
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
# build navigation2
colcon build
# source nav2 
echo 'source ~/work/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

# run
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/home/chunyu/work/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

# note
- ERROR: failed to solve: osrf/ros:humble-desktop: failed to do request: Head "https://registry-1.docker.io/v2/osrf/ros/manifests/humble-desktop": dial tcp [2a03:2880:f134:83:face:b00c:0:25de]:443: i/o timeout
    - 如果你所在的网络受限，可能需要配置 Docker 通过代理访问 Docker Hub。编辑 Docker 配置文件：
    ```
    sudo mkdir -p /etc/systemd/system/docker.service.d
    sudo nano /etc/systemd/system/docker.service.d/http-proxy.conf
    ```
    - 添加以下内容（替换为你的代理服务器地址和端口）：
    ```
    [Service]
    Environment="HTTP_PROXY=http://your-proxy-server:port/"
    Environment="HTTPS_PROXY=http://your-proxy-server:port/"
    ```
    - 保存后重新加载并重启 Docker：
    ```
    sudo systemctl daemon-reload
    sudo systemctl restart docker
    ```



rosdep update


# reference documentation
- nav2
nav2 Document ：https://docs.nav2.org/getting_started/index.html
- turtlebot3
https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble
https://github.com/ROBOTIS-GIT/turtlebot3_simulations