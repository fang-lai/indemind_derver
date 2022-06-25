#!/bin/bash

# Define colors
RCol="\e[0m"
Red="\e[0;31m"
Gre="\e[0;32m"
Yel="\e[0;33m"
Blu="\e[0;34m"

#export ROS_HOSTNAME=jon-y9000p
#export ROS_MASTER_URI=http://ubt-orin:11311

#cpu_arch=`arch`
#echo -e "${Red}cpu arch = $cpu_arch $RCol "
#sudo -s

export ROOT_PATH=$(
    cd $(dirname ${BASH_SOURCE:-$0})
    pwd
)

#source ${ROOT_PATH}/setup.bash 
#source  /opt/ros/noetic/setup.bash

${ROOT_PATH}/build/indemind_node
