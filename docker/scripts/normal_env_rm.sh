#!/usr/bin/env bash
echo "stop and rm docker" 
docker stop normal_env > /dev/null
docker rm -v -f normal_env > /dev/null