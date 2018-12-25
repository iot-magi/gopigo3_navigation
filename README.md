# How to run ROS navigation stack on gopigo3
## Introduction

This repository explains the way to mount a low price lidar on GoPiGo3 and run navigation stack with it.

## Setup instructions
1. Hardware setup
   1. Setup GoPiGo3  
   1. Add power supply circuit  
   1. Set up USB connection  
   1. Mount Lidar unit to GoPiGo3  
1. Software setup
   1. Notebook PC setup
      1. Install ROS
      1. ros network configration
      1. Install Packages
      1. ROS Package Build
      1. NTP setup
   1. GoPiGo setup
      1. Install ROS
      1. ros network configration
      1. Install Packages
      1. ROS Package Build
      1. NTP setup
1. Execution method

## Requirement
Hardware requirement
-  GoPiGo3 + Raspberry Pi3  
-  Notebook PC  
-  LIDAR X4 (YDLIDAR)  
-  5V output dc-dc converter board (< 2A)  
-  self power USB hub  
-  11.1V LiPo battery (not mandatory, original dry cell battery can be used)

<div align="center"><img src="images/S__10698773.jpg" width="400"></div>

Operating System Requirement
-  OS : Ubuntu 16.04 (Xenial) for Notebook PC
-  OS : Ubuntu mate 16.04 for GoPiGo3

## Result
Robot

<div align="center">
  <img src="images/IMG_0706.JPG" width="300">
</div>

Run Result

<div align="center">
  <img src="images/gopigo3_rviz_real(cut).gif" width="700">
</div>

# Hardware setup
##  Setup GoPiGo3  
Assenble GoPiGo3 from parts

<div align="center">
  <img src="images/S__10911746.jpg" width="300">
</div>

##  Add power supply circuit  
Battery -> dc-dc converter -> USB Hub power socket  
<div align="center">
  <img src="images/S__10698770.jpg" width="220">
  <img src="images/S__10698768.jpg" width="220">
  <img src="images/S__10698766.jpg" width="180">
</div>

##  Set up USB connection  
Lidar -> serial-USB converter board (comes with Lidar)  -> USB Hub -> Raspberry Pi  
<div align="center">
  <img src="images/S__10698759.jpg" width="300">
</div>

##  Mount Lidar unit to GoPiGo3  

<div align="center">
  <img src="images/S__10698756.jpg" width="300">
</div>

# Notebook PC setup
Install ROS to Note book PC

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install python-rosinstall

Create Catkin Workspace on Notebook PC

    mkdir -p ~/catkin_ws/src  
    cd ~/catkin_ws/src  
    catkin_init_workspace  
    cd ~/catkin_ws/  
    catkin_make  
    source devel/setup.bash

### ROS network configuration for Notebook PC

    export ROS_MASTER_URI=http://<IP Adress of Notebook PC>:11311
    export ROS_IP=<IP Adress of Notebook PC>

### Install ROS Packages to Notebook PC

    sudo apt-get install ros-kinetic-navigation
    sudo apt-get install ros-kinetic-gmapping

Install gopigo3_navigation package

    cd ~/catkin/src/
    git clone https://github.com/taityo/gopigo3_navigation.git    

### Build ROS Package on Notebook PC

    cd ~/catkin/
    catkin make

### NTP setup on Notebook PC
Install ntp

    sudo apt-get install ntp

ntp configration

    # /etc/ntp.conf

    # write file top
    tinker panic 0

    server (server name) iburst

# GoPiGo3 setup

### Install ROS to GoPiGo3 

Install ROS to GoPiGo3

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install python-rosinstall

Make Workspace on GoPiGo3  

    mkdir -p ~/catkin_ws/src  
    cd ~/catkin_ws/src  
    catkin_init_workspace  
    cd ~/catkin_ws/  
    catkin_make  
    source devel/setup.bash

## ROS network configuration of GoPiGo3 

    export ROS_MASTER_URI=http://<IP Adress of Notebook PC>:11311
    export ROS_IP=<IP Adress of GoPiGo3>

### Install gopigo3_node to GoPiGO3

※Please see [gopigo3_node](https://github.com/ros-gopigo/gopigo3_node) for more informagiton.

    cd ~/catkin/src/
    git clone https://github.com/ros-gopigo/gopigo3_node.git

rerwite gopigo3_driver.py

    # gopigo3_node/src/gopigo3_driver.py

    line 259  gopigo -> base_link
    line 259   world -> odom    

### Setup ydlidar package to GoPiGo3

※Please see [ydlidar](https://github.com/EAIBOT/ydlidar) for more informagiton.

    # ydlidar package clone
    cd ~/catkin/src/
    git clone https://github.com/EAIBOT/ydlidar.git

Install ydlidar driver

    # ydliar driver setup
    roscd ydlidar/startup
    sudo chmod 777 ./*
    sudo sh initenv.sh

Install gopigo3_controller  
※Please see [gopigo3_controller](https://gitlab.t-lab.cs.teu.ac.jp/MAGI/gopigo3_controller) for more informagiton.

    cd ~/catkin/src/
    git clone https://github.com/taityo/gopigo3_controller.git

### ROS Package Build on GoPiGo3

    cd ~/catkin/
    catkin make

### NTP setup on GoPiGo3

Install ntp

    sudo apt-get install ntp

ntp configration

    # /etc/ntp.conf

    # write file top
    tinker panic 0

    server (server name) iburst

# Execution

## Run gopigo3_navigation on Notebook PC
If you want to use rviz, you can run the following command.

    roslaunch gopigo3_navigation navigation_view.launch

If you want to run navigation stack by program, you can run the following command

    roslaunch gopigo3_navigation navigation.launch

## Run gopigo3_controller on GoPiGo3

※This launch file should be run both gopigo3_driver.py and ydlidar.launch

    roslaunch gopigo3_controller gopigo3.launch

# License

![BSD](http://img.shields.io/badge/license-BSD-green.svg)

Copyright (c) 2018, Tokyo Univercity of Technology

