
# 一 build docker images
- create ros images
```
cd {your_workspace}/ros2_test_env/docker/
docker build --network host -f base.dockerfile . -t ros_humble:latest
```
- add user
```
docker build -f create_user.dockerfile . -t ros_humble:chunyu
```
# 二 run docker container
```
./docker/scripts/normal_env_run.sh
```

# 三 add source 
## 1. download navigation2 && turtlebot3 src
```
git clone https://github.com/ros-navigation/navigation2.git --branch humble
git clone git@github.com:ROBOTIS-GIT/turtlebot3_simulations.git --branch humble
```
## 2. into docker container
```
./docker/scripts/normal_env_into.sh
```
## 3. add navigation2 dependences
```
sudo apt update
sudo apt install python3-rosdep
rosdep init
rosdep update
cd ~/work
rosdep install --from-paths src/navigation2 --ignore-src -r -y
```
## 4. source env
```
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
## 5. build navigation2
```
colcon build
```
## 6. source nav2 
```
echo 'source ~/work/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
## 7. run
```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/home/chunyu/work/src/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```



# 使用现有编程规则
- 如果希望使用现有的编程规则，可以克隆以下代码仓库
```
git clone git@github.com:wang-chun-yu/cursor-rules.git
ln -s git@github.com:wang-chun-yu/cursor-rules.git
// 可能需要重启cursor加载规则
// 新增chat窗口，询问“”你是谁，你有什么作用"检查规则是否加载
```


# note
## 1. ERROR: failed to solve: osrf/ros:humble-desktop: failed to do request: Head "https://registry-1.docker.io/v2/osrf/ros/manifests/humble-desktop": dial tcp [2a03:2880:f134:83:face:b00c:0:25de]:443: i/o timeout
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


# reference documentation
- nav2
nav2 Document ：https://docs.nav2.org/getting_started/index.html
- turtlebot3
https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble
https://github.com/ROBOTIS-GIT/turtlebot3_simulations