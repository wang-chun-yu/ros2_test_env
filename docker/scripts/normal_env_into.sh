#!/usr/bin/env bash
#启动一个docker容器；
xhost +local:root 1>/dev/null 2>&1
docker start normal_env 1>/dev/null 2>&1
docker exec \
    -it normal_env \
    /bin/bash 
xhost -local:root 1>/dev/null 2>&1
