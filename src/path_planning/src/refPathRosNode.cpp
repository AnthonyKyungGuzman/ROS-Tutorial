#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "Eigen/Dense"
#include <math.h>
 
int g_rate;
bool g_is_debug;
double g_freq;
double g_amplitude;
double g_phase;
double g_w;

Eigen::Vector3d g_imu_data;

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

Eigen::Vector3d CreatePath(void)
{
  Eigen::Vector3d reference;
  //GeneratePath();
  reference.x = g_amplitude*sin(g_w*g_imu_data.x + g_phase);
  reference.y = g_amplitude*cos(g_w*g_imu_data.x + g_phase);

  return reference;
}


void GetParams(ros::NodeHandle nh)
{
  nh.getParam("debug",g_is_debug);
  nh.getParam("rate",g_rate);
  nh.getParam("amplitude",g_amplitude);
  nh.getParam("frequency",g_freq);
  nh.getParam("phase",g_phase);

  g_w = 2*M_PI*g_freq;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_planning");
  ROS_INFO("Path planning node created \n");
  ros::NodeHandle node_nh("~");
  ros::NodeHandle node_n;

  GetParams(node_nh);
  ROS_INFO("Path planning node parameters initialized \n");

	if(g_is_debug)
	{
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	}else
	{
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	}
  
  ros::Subscriber imu_subs = node_n.subscribe("/car/imu", 1, ImuCallback);
  ros::Publisher ref_pub = node_n.advertise<geometry_msgs::PoseStamped>("/car/ref", 1);
   
  ros::Rate loop_rate(g_rate);

  geometry_msgs::PoseStamped ref_msg;

  ROS_INFO("Starting path planning node \n");
   
  while (ros::ok())
  {
    ros::spinOnce();

    Eigen::Vector3d reference = GetSensorData();
    ref_msg.header.stamp = ros::Time::now();
    ref_msg.pose.position.x = reference.x;
    ref_msg.pose.position.y = reference.y;
    ref_msg.pose.position.z = reference.z;

    ROS_DEBUG("Reference x: %d, y: %d, z: %d",reference.x, reference.y, reference.z);
    
    ref_pub.publish(ref_msg);
    loop_rate.sleep();
  }

  return 0;
}