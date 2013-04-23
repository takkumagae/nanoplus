#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <quadrotor_msgs/OpticalFlow.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Core>

double vicon_x,vicon_y,vicon_vx,vicon_vy,vicon_vz;
double current_t = 0,previous_t = 0,dt;
double h,d_roll,d_pitch,d_yaw,bias_vx,bias_vy;
double pos_x=0,pos_y=0;
int initial_flag;
double initial_x=0,initial_y=0;
Eigen::Quaterniond q;
Eigen::Vector3d v,current_v,previous_v1,previous_v2;
ros::Publisher filter_pub;

void sensorCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	
  Eigen::Matrix3d R(q); 
	quadrotor_msgs::OpticalFlow vel;
	vel.vicon_vx = vicon_vx;
	vel.vicon_vy = vicon_vy;
	vel.raw_vx = msg->vector.x/30;
	vel.raw_vy = msg->vector.y/30;
	bias_vx = d_pitch*h*0.2;
	bias_vy = d_roll*h*0.2;
	current_v(0) = msg->vector.x/30*h+bias_vx;
	current_v(1) = msg->vector.y/30*h+bias_vy; 
	current_v(2) = vicon_vz;
	v = (current_v+previous_v1+previous_v2)/3;
	previous_v2 = previous_v1;
	previous_v1 = current_v;
	
 // v = R*v;	
	previous_t = current_t;
	current_t = ros::Time::now().toSec();
	dt = current_t-previous_t;
	pos_x = pos_x+dt*v(0);
	pos_y = pos_y+dt*v(1);
  pos_x = initial_x+pos_x;
	pos_y = initial_y+pos_y;
	vel.bias_vx = bias_vx;
	vel.bias_vy = bias_vy;
	vel.vicon_x = vicon_x;
	vel.vicon_y = vicon_y;
	vel.x = current_v(0);
	vel.y = previous_v2(0);
	vel.vx = v(0);
	vel.vy = v(1);
	filter_pub.publish(vel);
}

void viconCallback(const nav_msgs::Odometry::ConstPtr & vicon){
	vicon_vx = vicon->twist.twist.linear.x;
	vicon_vy = vicon->twist.twist.linear.y;
	vicon_vz = vicon->twist.twist.linear.z;
	q.w() = vicon->pose.pose.orientation.w;
	q.x() = vicon->pose.pose.orientation.x;
	q.y() = vicon->pose.pose.orientation.y;
	q.z() = vicon->pose.pose.orientation.z;
	h = vicon->pose.pose.position.z-0.6;
	if(initial_flag==0)
	{
		initial_x = vicon->pose.pose.position.x;
		initial_y = vicon->pose.pose.position.y;
		initial_flag = 1;
	}
  d_roll = vicon->twist.twist.angular.x;
	d_pitch = vicon->twist.twist.angular.y;
	d_yaw = vicon->twist.twist.angular.z;
	vicon_x = vicon->pose.pose.position.x;
	vicon_y = vicon->pose.pose.position.y;
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"filter");
	ros::NodeHandle n;
	ROS_INFO("ready....");
  previous_v1 = Eigen::Vector3d::Zero();
	previous_v2 = Eigen::Vector3d::Zero();
	ros::Subscriber sensor_sub=n.subscribe("/adns3080/optical_flow",10,sensorCallback);
	ros::Subscriber vicon_sub=n.subscribe("/vicon",10,viconCallback);
	filter_pub = n.advertise<quadrotor_msgs::OpticalFlow>("optical_flow",10);
	ros::spin();
	return 0;
}
