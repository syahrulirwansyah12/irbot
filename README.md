# Irbot
ITB student's final project to create a simple SLAM Robot

# Preparation
1. Prepare the Telkomsel 4G LTE Modem and plug it into electricity to turn it on.
2. Connect a computer to Telkomsel 4G LTE Modem network named "Irbot_slam".
3. Turn on the Raspberry Pi switch in the mobile robot and wait until it is connected to the "Irbot_slam" network.
4. Connect to the mobile robot from a computer that has been connected to the "Irbot_slam" network by entering this command into the computer's terminal.
```
ssh -X irbot@192.168.43.59
```
5. Then enter the mobile robot's password.

# Running program in Raspberry Pi
1. Connect the RPLidar USB cable to the Raspberry Pi USB port.
2. From the computer's terminal that has been connected to the mobile robot via ssh, enter this command to start the RPLidar scan.
```
roslaunch irbot irbot_rplidar.launch
```
4. Open the other computer's terminal and connect to the mobile robot via ssh. Enter this command to connect a PS4 joystick to Raspberry Pi.
```
startds4
```
6. Prepare a PS4 joystick, then press the "PS" button and the "share" button simultaneously until the LED is blinking rapidly. Wait until the LED turns to solid blue.
7. Open the other computer's terminal and connect to the mobile robot via ssh. Enter this command to send the velocity command from the joystick to Arduino.
```
roslaunch irbot irbot_teleop_joy.launch
```
9. Connect the Arduino USB cable to the Raspberry Pi port.
10. Open the other computer's terminal again and connect to the mobile robot via ssh. Then, enter this command to set the Arduino to receive the velocity command.
```
roslaunch irbot irbot_serial_node.launch
```
12. Connect the USB-TTL converter to the Raspberry Pi port and Arduino RX3 and TX3.
13. Open another computer's terminal, then connect to the mobile robot via ssh again. After that, enter this command to send the odometry data from Arduino to Raspberry Pi.
```
rosrun irbot serial_arduino.py
```

# Running program in Ubuntu computer
1. Prepare a computer that has Ubuntu operating system and ROS already installed.
2. Connect the Ubuntu computer to the "Irbot_slam" network, then check the IP address of the computer that connects to the "Irbot_slam" local network by entering this command in the Ubuntu terminal.
```
hostname -I
```
3. Edit the `.bashrc` file by entering this command in the Ubuntu terminal.
```
nano .bashrc
```
4. Add this line to the end of the file.
```
export ROS_IP = $LOCAL_IP
export ROS_HOSTNAME = $LOCAL_IP
export ROS_MASTER_URI = http://$ROBOT_IP:11311
```
5. Replace `$LOCAL_IP` with the IP shown in `hostname -I`. It should be some kind of this value `192.168.43.XX`.
6. Replace `$ROBOT_IP` with `192.168.43.59` as it is the robot IP address in "Irbot_slam" local network.
7. Step 3 to 6 are only needed to be done once.
8. Open the Ubuntu terminal then enter this command to run the SLAM program in RViz.
```
roslaunch irbot_rviz slam_karto.launch
```
