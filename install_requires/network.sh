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
<<<<<<< HEAD
sh -c "sudo echo \"                    password: \"1234567890\"\" >> /etc/netplan/50-cloud-init.yaml"
=======
sh -c "echo \"                    password: \"1234567890\"\" >> /etc/netplan/50-cloud-init.yaml"
>>>>>>> e5ca09e50ce68673b9f9193c22cd34399ac59546
sh -c "echo \"        wlan0:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            dhcp4: true\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            optional: true\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"            access-points:\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"                \"River:\"\" >> /etc/netplan/50-cloud-init.yaml"
sh -c "echo \"                    password: \"tothemoooon\"\" >> /etc/netplan/50-cloud-init.yaml"

sudo netplan generate
sudo netplan apply
