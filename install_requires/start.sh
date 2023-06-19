#!/bin/bash

echo "[Update the package lists and upgrade them]"
sudo apt-get update -y


echo "[Install and enable SSH]"
sudo apt-get install openssh-server
sudo systemctl enable ssh.service
sudo systemctl start ssh.service
sudo dpkg-reconfigure openssh-server


echo "[Install X2Go Server]"
sudo apt-get install software-properties-common -y
sudo add-apt-repository ppa:x2go/stable -y
sudo apt-get install x2goserver s2goserver-xsession
sudo apt-get install x2gomatebindings


echo "[Install ROS]"
cd /home/$USER
git clone https://github.com/ROBOTIS-GIT/robotis_tools.git


echo "[Join ZeroTier Network]"
curl -s https://install.zerotier.com | sudo bash
sudo zerotier-cli join e5cd7a9e1cfb25b9
