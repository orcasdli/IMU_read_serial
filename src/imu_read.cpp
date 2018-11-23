#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <imu_read/JY901.h>

const double pi = 3.1415926536;

int counter = 0;

double Linear_acc[3] = {0, 0, 0};
double RPY_angle[3] = {0, 0, 0};
double Omega[3] = {0, 0, 0};


sensor_msgs::Imu imu;
tf::Quaternion imu_quaternion;
int main(int argc, char **argv) {

  serial::Serial ser;
  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string imu_frame_id;
  double time_offset_in_seconds;

  tf::Quaternion zero_orientation;

  ros::init(argc, argv, "imu_read");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");

  private_node_handle.param<std::string>("imu_frame_id", imu_frame_id,
                                         "imu_base");
  private_node_handle.param<double>("time_offset_in_seconds",
                                    time_offset_in_seconds, 0.0);

  ros::NodeHandle nh;
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);


  ros::Rate r(40); // 1000 hz

  unsigned char chrBuffer[33];
  unsigned char chrTemp[33];
  unsigned short usLength = 0, usRxLength = 0;

  while (ros::ok()) {
    try {
      if (ser.isOpen()) {
        // read string from serial device
        if (ser.available()) {

          usLength = ser.read(chrBuffer, 33);
            printf("usLength!!!! = %d\n", usLength);

		if (usLength>0)
		{
			JY901.CopeSerialData(chrBuffer,usLength);
		}

    	printf("Time:20%d-%d-%d %d:%d:%.3f\r\n",(short)JY901.stcTime.ucYear,(short)JY901.stcTime.ucMonth,
					(short)JY901.stcTime.ucDay,(short)JY901.stcTime.ucHour,(short)JY901.stcTime.ucMinute,(float)JY901.stcTime.ucSecond+(float)JY901.stcTime.usMiliSecond/1000);

			printf("Acc:%.3f %.3f %.3f\r\n",(float)JY901.stcAcc.a[0]/32768*16,(float)JY901.stcAcc.a[1]/32768*16,(float)JY901.stcAcc.a[2]/32768*16);

			printf("Gyro:%.3f %.3f %.3f\r\n",(float)JY901.stcGyro.w[0]/32768*2000,(float)JY901.stcGyro.w[1]/32768*2000,(float)JY901.stcGyro.w[2]/32768*2000);

			printf("Angle:%.3f %.3f %.3f\r\n",(float)JY901.stcAngle.Angle[0]/32768*180,(float)JY901.stcAngle.Angle[1]/32768*180,(float)JY901.stcAngle.Angle[2]/32768*180);

			printf("Mag:%d %d %d\r\n",JY901.stcMag.h[0],JY901.stcMag.h[1],JY901.stcMag.h[2]);

			printf("Pressure:%lx Height%.2f\r\n",JY901.stcPress.lPressure,(float)JY901.stcPress.lAltitude/100);

			printf("DStatus:%d %d %d %d\r\n",JY901.stcDStatus.sDStatus[0],JY901.stcDStatus.sDStatus[1],JY901.stcDStatus.sDStatus[2],JY901.stcDStatus.sDStatus[3]);

			printf("Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",JY901.stcLonLat.lLon/10000000,(double)(JY901.stcLonLat.lLon % 10000000)/1e5,JY901.stcLonLat.lLat/10000000,(double)(JY901.stcLonLat.lLat % 10000000)/1e5);

			printf("GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n",(float)JY901.stcGPSV.sGPSHeight/10,(float)JY901.stcGPSV.sGPSYaw/10,(float)JY901.stcGPSV.lGPSVelocity/1000);
		

        // calculate measurement time
        ros::Time measurement_time =
            ros::Time::now() + ros::Duration(time_offset_in_seconds);

        // publish imu message

        imu.header.stamp = measurement_time;
        imu.header.frame_id = imu_frame_id;

        imu_quaternion.setRPY((float)JY901.stcAngle.Angle[0]/32768/pi,(float)JY901.stcAngle.Angle[1]/32768/pi,(float)JY901.stcAngle.Angle[2]/32768/pi); 
        imu.orientation.w = imu_quaternion.getW();
imu.orientation.x = imu_quaternion.getX();
imu.orientation.y = imu_quaternion.getY();
imu.orientation.z = imu_quaternion.getZ();

        imu.linear_acceleration.x = (float)JY901.stcAcc.a[0] / 32768 * 16;
        imu.linear_acceleration.y = (float)JY901.stcAcc.a[1] / 32768 * 16;
        imu.linear_acceleration.z = (float)JY901.stcAcc.a[2] / 32768 * 16;

        imu.angular_velocity.x = (float)JY901.stcGyro.w[0] / 32768 * 2000;
        imu.angular_velocity.y = (float)JY901.stcGyro.w[1] / 32768 * 2000;
        imu.angular_velocity.z = (float)JY901.stcGyro.w[2] / 32768 * 2000;

                  // i do not know the orientation covariance
                  imu.orientation_covariance[0] = 0;
                  imu.orientation_covariance[1] = 0;
                  imu.orientation_covariance[2] = 0;
                  imu.orientation_covariance[3] = 0;
                  imu.orientation_covariance[4] = 0;
                  imu.orientation_covariance[5] = 0;
                  imu.orientation_covariance[6] = 0;
                  imu.orientation_covariance[7] = 0;
                  imu.orientation_covariance[8] = 0;

       
                  imu.angular_velocity_covariance[0] = 0;
                  imu.angular_velocity_covariance[1] = 0;
                  imu.angular_velocity_covariance[2] = 0;
                  imu.angular_velocity_covariance[3] = 0;
                  imu.angular_velocity_covariance[4] = 0;
                  imu.angular_velocity_covariance[5] = 0;
                  imu.angular_velocity_covariance[6] = 0;
                  imu.angular_velocity_covariance[7] = 0;
                  imu.angular_velocity_covariance[8] = 0;
   
                  imu.linear_acceleration_covariance[0] = 0;
                  imu.linear_acceleration_covariance[1] = 0;
                  imu.linear_acceleration_covariance[2] = 0;
                  imu.linear_acceleration_covariance[3] = 0;
                  imu.linear_acceleration_covariance[4] = 0;
                  imu.linear_acceleration_covariance[5] = 0;
                  imu.linear_acceleration_covariance[6] = 0;
                  imu.linear_acceleration_covariance[7] = 0;
                  imu.linear_acceleration_covariance[8] = 0;

                  imu_pub.publish(imu);

        } else // ser not available
        {
        }
      } else {
        // try and open the serial port
        try {
          ser.setPort(port);
          ser.setBaudrate(115200);
          // ser.setBaudrate(9600);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
 printf("？？？？counter = %d\n", counter);

        } catch (serial::IOException &e) {
          ROS_ERROR_STREAM("Unable to open serial port "
                           << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if (ser.isOpen()) {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized.");
        } else {
          // ROS_INFO_STREAM("Could not initialize serial port.");
        }
      }
    } catch (serial::IOException &e) {
      ROS_ERROR_STREAM("Error reading from the serial port "
                       << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    printf("counter = %d\n", counter);
    counter++;
    r.sleep();
  }
}
