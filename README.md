# IMU_read_serial
Read IMU info from IMU device (customed, for example cheap IMU using MPU6050 or MPU9250)through a serial com port. Currently using the serial package in ROS.

I am using a cheap customed IMU with MPU9250 inside. It send IMU info through a serial com.
I bought it from Taobao (yeah, made in China). The device is called JY901. The orginal examples do not contain a Linux example. This project can be used to similar devices.


# How to use
Actually you should have known these after some basic ROS learning. Make sure you can access the com port. Make sure you know the right com port name such as "ttyUSB0"

1. Make your own ROS workspace. Install the ROS serial package by 
`sudo apt-get install ros-<distro>-serial`

for example
`sudo apt-get install ros-kinetic-serial`

2. Switch to the src file,
for example
`cd ~/catkin_ws/src`

git clone this into it:
`git clone https://github.com/orcasdli/IMU_read_serial.git`

### These files should be in a directory named as "imu_read". I cannot change name of this git clone anymore. So please change it manually.

3. Some traditional work in ROS.

for example

`cd ~/catkin_ws`

`catkin_make`

then
switch to the launch directory and 
`roslaunch imu_read.launch`

## Similar project:
https://github.com/kintzhao/imu_serial_node

and my fork:
https://github.com/orcasdli/imu_serial_node

### Personal thoughts:

I find it difficult to use serial com smoothly in Linux because we lack some mature libs, such as pccomm in Windows.
Therefore, this project currently forcefully reads packages from the buffer pool. If the buffer size and freqency do not match, it will not work well. Current it works by my blind guess (I feel kinda sad of my limited brain size).

If you have some better solution for serial comunication in Linux (can be customed), please let me know. Many thanks! 

Contact me if necessary: lizhi_haligong@163.com
