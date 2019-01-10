# How to run ROS navigation stack on [GoPiGo3](https://www.dexterindustries.com/gopigo3/)
## Introduction

This repository explains the way to mount a low cost lidar sensor on [GoPiGo3](https://www.dexterindustries.com/gopigo3/) and run the ROS navigation stack with it.

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
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

### ROS network configuration for Notebook PC

    export ROS_MASTER_URI=http://<IP Adress of Notebook PC>:11311
    export ROS_IP=<IP Adress of Notebook PC>

### Install ROS Packages to Notebook PC

    sudo apt-get install ros-kinetic-navigation
    sudo apt-get install ros-kinetic-gmapping

Install gopigo3_navigation package

    cd ~/catkin/src/
    git clone https://github.com/iot-magi/gopigo3_navigation.git

### Build ROS Package on Notebook PC

    cd ~/catkin/
    catkin_make

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
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

## ROS network configuration of GoPiGo3 

    export ROS_MASTER_URI=http://<IP Adress of Notebook PC>:11311
    export ROS_IP=<IP Adress of GoPiGo3>

### Install gopigo3_node to GoPiGO3

Note: Please see [gopigo3_node](https://github.com/ros-gopigo/gopigo3_node) for more informagiton.

    cd ~/catkin/src/
    git clone https://github.com/ros-gopigo/gopigo3_node.git

rerwite gopigo3_driver.py

    # gopigo3_node/src/gopigo3_driver.py

    line 106 "odometry" -> "odom"
    line 259    world -> odom    
    line 259 gopigo -> base_link

### Setup ydlidar package to GoPiGo3

Note: Please see [ydlidar](https://github.com/EAIBOT/ydlidar) for more informagiton.

    # ydlidar package clone
    cd ~/catkin/src/
    git clone https://github.com/EAIBOT/ydlidar.git

Install ydlidar driver

    # ydliar driver setup
    cd ydlidar/startup
    sudo chmod 777 ./*
    sudo sh initenv.sh

Make nav_lidar.launch

    # ydlidar/launch/nav_lidar.launch

    <launch>
      <node name="ydlidar_node"  pkg="ydlidar"  type="ydlidar_node" output="screen" respawn="false" >
        <param name="port"         type="string" value="/dev/ydlidar"/>  
        <param name="baudrate"     type="int"    value="115200"/>
        <param name="frame_id"     type="string" value="lidar"/>
        <param name="low_exposure"  type="bool"   value="false"/>
        <param name="resolution_fixed"    type="bool"   value="true"/>
        <param name="auto_reconnect"    type="bool"   value="true"/>
        <param name="reversion"    type="bool"   value="false"/>
        <param name="angle_min"    type="double" value="-180" />
        <param name="angle_max"    type="double" value="180" />
        <param name="range_min"    type="double" value="0.1" />
        <param name="range_max"    type="double" value="16.0" />
        <param name="ignore_array" type="string" value="" />
        <param name="samp_rate"    type="int"    value="9"/>
        <param name="frequency"    type="double" value="7"/>
      </node>
    </launch>

### ROS Package Build on GoPiGo3

    cd ~/catkin/
    catkin_make

### NTP setup on GoPiGo3

Install ntpdate

    sudo apt-get install ntpdate

Run ntpdate

    ntpdate (server name)

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

If you want to run navigation stack by program, you can run the following command.

    roslaunch gopigo3_navigation navigation.launch

## Run launchfile on GoPiGo3
Run gopigo3_node

    roslaunch gopigo3_node gopigo3.launch

Run ydlidar  
Note: Run this launchfile on another terminal.

    roslaunch ydlidar nav_lidar.launch

# License

![BSD](http://img.shields.io/badge/license-BSD-green.svg)

Copyright (c) 2018, Tokyo Univercity of Technology

