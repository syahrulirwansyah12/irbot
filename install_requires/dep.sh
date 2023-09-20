#!/bin/bash

echo "[Install ROS Dependencies]"
sudo apt-get install -y ros-melodic-rosserial-arduino ros-melodic-rosserial
sudo apt-get install -y ros-melodic-rplidar-ros
sudo apt-get install -y ros-melodic-slam-karto ros-melodic-hector-slam ros-melodic-gmapping ros-melodic-amcl ros-melodic-map-server
sudo apt-get install -y ros-melodic-move-base ros-melodic-dwa-local-planner ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-topic-tools ros-melodic-explore-lite
sudo apt-get install -y ros-melodic-joy ros-melodic-teleop-*

