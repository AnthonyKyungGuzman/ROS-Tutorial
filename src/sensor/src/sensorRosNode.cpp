#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "eigen3/Eigen/Dense"
 
int g_rate;
bool g_is_debug;

Eigen::Vector3d GetSensorData(void)
{
  static Eigen::Vector3d data(1, -1, 0);
  Eigen::Vector3d imu_data;
  //ReadImu();
  imu_data.x() = data.x();
  imu_data.y() = data.y();
  imu_data.z() = data.z();

  data.x() += 1;
  data.y() -= 1; 
  return imu_data;
}

void GetParams(ros::NodeHandle nh)
{
  nh.getParam("debug",g_is_debug);
  nh.getParam("rate",g_rate);
}


int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "sensor");
  ROS_INFO("IMU node created \n");
  ros::NodeHandle node_nh("~");
  ros::NodeHandle node_n;

  GetParams(node_nh);
  ROS_INFO("IMU node parameters initialized \n");

	if(g_is_debug)
	{
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	}else
	{
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	}
  
  ros::Publisher imu_pub = node_n.advertise<sensor_msgs::Imu>("/car/imu", 1);
   
  ros::Rate loop_rate(g_rate);

  sensor_msgs::Imu imu_msg;

  ROS_INFO("Starting imu node \n");
   
  while (ros::ok())
  {
    
    Eigen::Vector3d imu_data = GetSensorData();
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.linear_acceleration.x = imu_data.x();
    imu_msg.linear_acceleration.y = imu_data.y();
    imu_msg.linear_acceleration.z = imu_data.z();

    ROS_DEBUG("IMU Data x: %f, y: %f, z: %f",imu_data.x(), imu_data.y(), imu_data.z());
    
    imu_pub.publish(imu_msg);
    loop_rate.sleep();
  }

  return 0;
}