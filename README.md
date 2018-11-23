# IMU_read_serial
Read IMU info from IMU device (customed, for example cheap IMU using MPU6050 or MPU9250)through a serial com. Currently using the serial package in ROS.

I am using a cheap customed IMU with MPU9250 inside. It send IMU info through a serial com.
I bought it from Taobao (yeah, made in China). The device is called JY901. The orginal exmaples do not contain a Linux example. This project can be used in similar devices.




## Similar project:
https://github.com/kintzhao/imu_serial_node

and my fork:
https://github.com/orcasdli/imu_serial_node

### Personal thoughts:

I find it difficult to use serial smoothly in Linux because we lack some mature libs, such as pccomm in Windows.
Therefore, this project currently forcefully reads packages from the buffer pool. If the buffer size and freqency do not match, it will not work well. Current it works by my blind guess (I feel kinda sad of my limited brain size).

If you have some better solution for serial comunication in Linux (can be customed), please let me know. Many thanks! 
