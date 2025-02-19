[TOC]
# build docker images
- create ros images
docker build --network host -f base.dockerfile . -t ros_humble:latest
- add user
docker build --network host -f create_user.dockerfile . -t ros_humble:chunyu
# pull repositories
```
git@github.com:ros-navigation/navigation2.git
git checkout humble
```
# install depends
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