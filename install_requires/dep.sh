#!/bin/bash

echo "[Install ROS Dependencies]"
sudo apt-get install -y ros-melodic-rplidar-ros
sudo apt-get install -y ros-melodic-slam-karto ros-melodic-hector-slam ros-melodic-gmapping
sudo apt-get install -y ros-melodic-move-base
sudo apt-get install -y ros-melodic-dwa-local-planner
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-explore-lite
sudo apt-get install -y ros-melodic-topic-tools

