#!/bin/bash

echo "[Update the package lists and upgrade them]"
sudo apt-get update && sudo apt-get upgrade -y

echo "[Install python3 and pip3]"
sudo apt-get install python3
sudo apt-get install python3-pip

echo "[Install ZeroTier]"
curl -s https://install.zerotier.com | sudo bash

# Install ROS
echo "[Install ROS]"
while true; do
    echo ""
    read -p "Do you want to install ROS on your computer? (Y/n) " yn_ros
    case $yn_ros in
    [Yy]*)
        cd /home/$USER
        git clone https://github.com/ROBOTIS-GIT/robotis_tools.git
        while true; do
            echo ""
            echo "These are the list of OS you might use:"
            echo "1) Ubuntu 18.04"
            echo "2) Ubuntu 20.04"
            echo "3) Cancel."
            echo ""
            read -p "Input the number --> " computer_os
            case $computer_os in
            [1]*)
                echo "[Install ROS Melodic (desktop full) on Ubuntu 18.04]"
                cd /home/$USER/robotis_tools/
                sudo chmod +x install_ros_melodic.sh
                sudo ./install_ros_melodic.sh
                break;;
            [2]*)
                echo "[Install ROS Melodic (desktop full) on Ubuntu 20.04]"
                cd /home/$USER/robotis_tools/
                sudo chmod +x install_ros_noetic.sh
                sudo ./install_ros_noetic.sh
                break;;
            [3]*)
                echo "OK. You are canceling the installation of ROS."
                echo "Removing [robotis_tools]."
                rm -r /home/$USER/robotis_tools
                break;;
            *)
                echo ""
                echo "Invalid input. Try again.";;
            esac
        done
        break;;
    [Nn]*)
        echo "OK."
        echo "Please refer to ROS Wiki (http://wiki.ros.org/ROS/Installation) to install ROS by yourself."
        break;;
    *)
        echo ""
        echo "Invalid input. Try again.";;
    esac
done

# Install ROS Melodic dependencies
echo "[Install ROS Dependencies]"
while true; do
    echo ""
    read -p "Do you want to install ROS package dependencies? (Only works in ROS Melodic) (Y/n)" yn_dep
    case $yn_dep in
    [Yy]*)
        sudo chmod +x dep.sh
        sudo ./dep.sh || { echo "ROS package dependencies installation is failed. Check your device storage or network connection."; exit 1;}
        break;;
    [Nn]*)
        echo "OK."
        break;;
    *)
        echo ""
        echo "Invalid input. Try again.";;
    esac
done