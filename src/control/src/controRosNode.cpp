#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "Eigen/Dense"
 
int g_rate;
bool g_is_debug;


Eigen::Vector3d g_imu_data;
Eigen::Vector3d g_ref;


void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_DEBUG("IMU Received Data x: %d, y: %d, z: %d",
    msg.linear_acceleration.x, 
    msg.linear_acceleration.y, 
    msg.linear_acceleration.z);
  g_imu_data.x = msg.linear_acceleration.x;
  g_imu_data.y = msg.linear_acceleration.y;
  g_imu_data.z = msg.linear_acceleration.z;
}

void RefCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_DEBUG("IMU Received Data x: %d, y: %d, z: %d",
    msg.pose.position.x, 
    msg.pose.position.y, 
    msg.pose.position.z);
  g_ref.x = msg.pose.position.x;
  g_ref.y = msg.pose.position.y;
  g_ref.z = msg.pose.position.z;
}


void GetParams(ros::NodeHandle nh)
{
  nh.getParam("debug",g_is_debug);
  nh.getParam("rate",g_rate);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ROS_INFO("Control node created \n");
  ros::NodeHandle node_nh("~");
  ros::NodeHandle node_n;

  GetParams(node_nh);
  ROS_INFO("Control node parameters initialized \n");

	if(g_is_debug)
	{
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	}else
	{
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	}
  
  ros::Subscriber imu_subs = node_n.subscribe("/car/imu", 1, ImuCallback);
  ros::Subscriber ref_subs = node_n.subscribe("/car/ref", 1, RefCallback);
   
  ros::Rate loop_rate(g_rate);

  ROS_INFO("Starting control node \n");
   
  while (ros::ok())
  {
    ros::spinOnce();

    //Control();
    
    loop_rate.sleep();
  }

  return 0;
}