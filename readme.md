[TOC]
# build docker images
- create ros images
docker build --network host -f base.dockerfile . -t ros_humble:latest
- add user
docker build --network host -f create_user.dockerfile . -t ros_humble:chunyu

# ~~build navigation2~~
```
export ROS_DISTRO=humble
git clone https://github.com/ros-navigation/navigation2.git --branch main
docker build --tag navigation2:$ROS_DISTRO   --build-arg FROM_IMAGE=ros:$ROS_DISTRO   --build-arg OVERLAY_MIXINS="release ccache lld"   --cache-from ghcr.io/ros-navigation/navigation2:main   ./navigation2 --network host --no-cache 
```
# download navigation2 src
git clone https://github.com/ros-navigation/navigation2.git --branch humble
git checkout humble
# add navigation dependences
sudo apt update
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# run

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/home/chunyu/work/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False


```
sudo apt update
#sudo apt install ros-humble-geographic-msgs
#sudo apt install ros-humble-bond-core
sudo apt install ros-humble-bondcpp
sudo apt install ros-humble-test-msgs
sudo apt-get install ros-humble-behaviortree-cpp-v3
sudo apt-get install libgraphicsmagick1-dev libgraphicsmagick++1-dev
sudo apt-get install ros-humble-diagnostic-updater
sudo apt-get install libceres-dev
sudo apt-get install libxtensor-dev
sudo apt-get install libompl-dev
```

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