#!/bin/bash

echo "[Update the package lists and upgrade them]"
sudo apt-get update && sudo apt-get upgrade -y

echo "[Install and enable SSH]"
sudo apt-get install openssh-server
sudo systemctl enable ssh.service
sudo systemctl start ssh.service
sudo dpkg-reconfigure openssh-server

echo "[Install ZeroTier]"
curl -s https://install.zerotier.com | sudo bash

echo "[Install python3 and pip3]"
sudo apt-get install python3
sudo apt-get install python3-pip

echo "[Install DS4 Driver]"
sudo pip3 install -y ds4drv
sudp pip3 install pyserial

# Install X2Go Server to robot machine
echo "[Install X2Go Server]"
while true; do
    echo ""
    read -p "Do you want to install X2Go Server? (Y/n) " yn_x2go
    case $yn_x2go in 
    [Yy]*)
        echo "[Install X2Go Server]"
        sudo apt-get install software-properties-common -y
        sudo add-apt-repository ppa:x2go/stable
        sudo apt-get update && sudo apt-get upgrade -y
        sudo apt-get install x2goserver s2goserver-xsession
        sudo apt-get install x2gomatebindings
        break;;
    [Nn]*)
        echo "OK"
        break;;
    *)
        echo ""
        echo "Invalid input. Try again.";;
    esac
done

# Connect the robot machine to ZeroTier Network
echo "[Connect to ZeroTier Network]"
while true; do
    read -p "Do you want to join a ZeroTier Network? (Y/n) " yn_zero
    case $yn_zero in
    [Yy]*)
        echo "[Join ZeroTier Network]"
        read -p "Insert your network ID" zerotier_network_id
        sudo zerotier-cli join $zerotier_network_id
        break;;
    [Nn]*)
        echo "OK"
        break;;
    *)
        echo ""
        echo "Invalid input. Try again.";;
    esac
done

# Install ROS
echo "[Install ROS]"
while true; do
    echo ""
    read -p "Do you want to install ROS on your robot machine? (Y/n) " yn_ros
    case $yn_ros in
    [Yy]*)
        cd /home/$USER
        git clone https://github.com/ROBOTIS-GIT/robotis_tools.git
        while true; do
            echo ""
            echo "These are the list of machines that you might used on your robot"
            echo "Robot machine used:"
            echo "1) RPi 3 with Ubuntu 18.04"
            echo "2) RPi 4 with Ubuntu 20.04"
            echo "3) Ubuntu 18.04 (e.g. Jetson Nano)"
            echo "4) Ubuntu 20.04 (e.g. Jetson Xavier)"
            echo "5) Cancel."
            echo ""
            read -p "Input the number --> " robot_machine
            case $robot_machine in
            [1]*)
                echo "[Install ROS Melodic (base) on RPi 3 with Ubuntu 18.04]"
                cd /home/$USER/robotis_tools/
                sudo chmod +x install_ros_melodic_rpi.sh
                sudo ./install_ros_melodic_rpi.sh
                break;;
            [2]*)
                echo "[Install ROS Melodic (base) on RPi 4 with Ubuntu 20.04]"
                cd /home/$USER/robotis_tools/
                sudo chmod +x install_ros_noetic_rpi.sh
                sudo ./install_ros_noetic_rpi.sh
                break;;
            [3]*)
                echo "[Install ROS Melodic (desktop full) on Ubuntu 18.04]"
                cd /home/$USER/robotis_tools/
                sudo chmod +x install_ros_melodic.sh
                sudo ./install_ros_melodic.sh
                break;;
            [4]*)
                echo "[Install ROS Melodic (desktop full) on Ubuntu 20.04]"
                cd /home/$USER/robotis_tools/
                sudo chmod +x install_ros_noetic.sh
                sudo ./install_ros_noetic.sh
                break;;
            [5]*)
                echo "OK. You are canceling the installation of ROS on your robot machine."
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

# Connect the robot machine to WiFi
echo "[Connect to Wireless Network]"
while true; do
    echo ""
    read -p "Do you want to connect your robot to a specific network now? (Y/n) " yn_net
    case $yn_net in
    [Yy]*)
        while true; do
            echo "Which method do you want to do:"
            echo "1) Network Manager CLI (nmcli)"
            echo "2) Netplan"
            echo "3) Cancel."
            read -p "Enter the method you prefer: " method_net
            case $method_net in
            [1]*)
                sudo apt-get install network-manager
                sudo nmcli device wifi rescan
                read -p "Insert SSID: " ssid_net
                read -p "Insert Password: " pass_net
                sudo nmcli device wifi connect \"$ssid_net_net\" password \"$pass_net\"
                echo "Done."
                break;;
            [2]*)
                sudo chmod +x network.sh
                sudo ./network.sh
                break;;
            [3]*)
                echo "OK."
                echo "You can connect the robot machine to a specific network later."
                break;;
            *)
                echo ""
                echo "Invalid input. Try again.";;
            esac
        done
        break;;
    [Nn]*)
        echo "OK."
        break;;
    *)
        echo ""
        echo "Invalid input. Try again.";;
    esac
done

# Autostart DS4DRV (need reference)

echo ""
echo "================================================"
echo ""
echo "Robot machine preparation is finished."
echo ""
echo "Goodluck with your project."
echo ""
echo "Reboot your machine."
echo ""
echo "================================================"
echo ""