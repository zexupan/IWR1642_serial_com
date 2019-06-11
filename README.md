serial_ti_radar
===========================
**UAV target tracking and pursuit using mmWave radar**

This repo provides a serial communication driver for TI IWR1642 mmWave sensor. This ROS package sends config parameters to a TI IWR1642 mmwave radar sensor, and then retrieve data about detected targets and publish them in PosewithCovariance messages.
This repo was tested in Ubuntu 16.04 and ROS kinetic.
To get it working, the TI IWR1642 has to be installed with the people counting demo firmware (tested using the firmware provided in mmwave_industrial_toolbox_2_3_0 lab0011-pplcount).
serial_ti_radar_config.cpp send the cfg command to the radar
serial_ti_radar_read.cpp extracts the radar info
frame_transformation.cpp UAV track and pursuit target in autonomous offboard mode

Installing
---------------------------
Clone this package and the serial package, then compile:

```
cd <your_catkin_ws_dir>/src
git clone https://github.com/wjwwood/serial.git
git clone https://github.com/jujuejundao/serial_ti_radar
cd ..
catkin_make
source devel/setup.bash
```

Launching
---------------------------
Before launching, make sure you have given permission to access the serial port (the two ports may show up different as /dev/ttyACM0 and /dev/ttyACM1, you may use ```ll /dev/serial/by-id/```to check the ports):

```
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1

```
To send the configs and start receiveing detected target information:
```
roslaunch serial_ti_radar serial_ti_radar_read.launch

```

Detailed Documentation
---------------------------
WIP
