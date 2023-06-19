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



echo "[Autoconnect Wifi]"
sh -c "echo \"network:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"    ethernets:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"        eth0:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            dhcp4: true\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            optional: true\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"    version: 2\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"    wifis:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"        rendered: networkd\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"        wlan0:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            dhcp4: true\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            optional: true\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            access-points:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"                \"Irbot_slam:\"\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"                    password: \"1234567890\"\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"        wlan0:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            dhcp4: true\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            optional: true\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            access-points:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"                \"River:\"\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"                    password: \"tothemoooon\"\" >> /etc/netplan/50-cloud-init.yaml"

sudo netplan generate
sudo netplan apply
