#!/bin/bash

echo ""
echo "You are going to configure Netplan to connect to wireless network."
echo ""
echo "Network configuration changes with this method can potentially lead to loss of connectivity."
echo ""
echo "If you proceed this further, you have to edit \"/etc/netplan/50-cloud-init.yaml\" file by yourself"
echo "when there is an error, typo, or changes in network name."
echo "Then save the file that you have edited and do"
echo "      \"sudo netplan generate\", and"
echo "      \"sudo netplan apply\"."
echo ""
echo "Please do \"ls /sys/class/net\" to find your wireless network interface."
echo "Your wireless network interface should be named as \"wlan0\" or something like \"wlp3s0\"."
while true; do
    echo ""
    read -p "Are you sure to proceed to configure Netplan? (Y/n) " yn_netplan
    case $yn_netplan in
    [Yy]*)
        echo "[Autoconnect Wifi using Netplan]"
        sh -c "echo \"network:\" >> /etc/netplan/50-cloud-init.yaml"
        sh -c "echo \"    ethernets:\" >> /etc/netplan/50-cloud-init.yaml"
        sh -c "echo \"        eth0:\" >> /etc/netplan/50-cloud-init.yaml"
        sh -c "echo \"            dhcp4: true\" >> /etc/netplan/50-cloud-init.yaml"
        sh -c "echo \"            optional: true\" >> /etc/netplan/50-cloud-init.yaml"
        sh -c "echo \"    version: 2\" >> /etc/netplan/50-cloud-init.yaml"
        sh -c "echo \"    wifis:\" >> /etc/netplan/50-cloud-init.yaml"
        sh -c "echo \"        renderer: networkd\" >> /etc/netplan/50-cloud-init.yaml"
        while true; do
            echo ""
            echo "Do you want to add another network to connect? (Y/n) " yn_add
            case $yn_add in
            [Yy]*)
                read -p "Insert your wireless network interface: " net_interface
                read -p "Insert your SSID: " net_ssid
                read -p "Insert your password: " pass
                sh -c "echo \"        $net_interface:\" >> /etc/netplan/50-cloud-init.yaml"
                sh -c "echo \"            dhcp4: true\" >> /etc/netplan/50-cloud-init.yaml"
                sh -c "echo \"            optional: true\" >> /etc/netplan/50-cloud-init.yaml"
                sh -c "echo \"            access-points:\" >> /etc/netplan/50-cloud-init.yaml"
                sh -c "echo \"                \"$net_ssid\":\" >> /etc/netplan/50-cloud-init.yaml"
                sh -c "echo \"                    password: \"$pass\"\" >> /etc/netplan/50-cloud-init.yaml"
                ;;
            [Nn]*)
                echo "OK."
                break;;
            *)
                echo ""
                echo "Invalid input. Try again.";;
            esac
        done
        sudo netplan generate || { echo "Error: Netplan configuration generation failed. Please edit \"/etc/netplan/50-cloud-init.yaml\" manually."; exit 1; }
        sudo netplan apply || { echo "Error: Netplan configuration application failed. Please edit \"/etc/netplan/50-cloud-init.yaml\" manually."; exit 1; }
        echo ""
        echo "Reboot your device if netplan is successfully generated and applied."
        break;;
    [Nn]*)
        echo "OK."
        break;;
    *)
        echo ""
        echo "Invalid input. Try again.";;
    esac
done
