#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <opticalflow_msgs/OpticalSensor.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <opticalflow_msgs/OpticalFlow.h>
#include <Eigen/Dense>
#include <Eigen/Core>

double vicon_x,vicon_y,vicon_z,vicon_vx,vicon_vy,vicon_vz;
double current_t = 0,previous_t = 0,dt;
double d_roll,d_pitch,d_yaw,bias_vx,bias_vy;
double pos_x=0,pos_y=0,h;
double ax,ay,az;
int initial_flag,initial_orien;
double initial_x=0,initial_y=0;
Eigen::Quaterniond q,q_imu;
Eigen::Vector3d v,current_v,previous_v;
ros::Publisher filter_pub;

void sensorCallback(const opticalflow_msgs::OpticalFlow::ConstPtr& msg)
{
	
  Eigen::Matrix3d R(q); 
  Eigen::Matrix3d R_imu(q_imu);
	opticalflow_msgs::OpticalSensor vel;
	h = msg->ground_distance;
	vel.vicon_vx = vicon_vx;
	vel.vicon_vy = vicon_vy;
  bias_vx = d_pitch*h;
  bias_vy = d_roll*h;
  v(0) = -sqrt(2)/2*msg->velocity_x-sqrt(2)/2*msg->velocity_y;
	v(1) = -sqrt(2)/2*msg->velocity_x+sqrt(2)/2*msg->velocity_y;
	v(2) = 0;
  v = R*v;	
	previous_t = current_t;
	current_t = ros::Time::now().toSec();
	dt = current_t-previous_t;
	pos_x = pos_x+dt*v(0);
	pos_y = pos_y+dt*v(1);
  pos_x = initial_x+pos_x;
	pos_y = initial_y+pos_y;
	vel.vicon_x = vicon_x;
	vel.vicon_y = vicon_y;
	vel.vicon_z = vicon_z;
	vel.x = bias_vx;
	vel.y = bias_vy;
	vel.z = msg->ground_distance;
	vel.vx = v(0);
	vel.vy = v(1);
	filter_pub.publish(vel);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu){
	q_imu.w() = imu->orientation.w;
	q_imu.x() = imu->orientation.x;
	q_imu.y() = imu->orientation.y;
	q_imu.z() = imu->orientation.z;
	d_roll = imu->angular_velocity.x;
	d_pitch = imu->angular_velocity.y;
	d_yaw = imu->angular_velocity.z;
	ax = imu->linear_acceleration.x;
	ay = imu->linear_acceleration.y;
	az = imu->linear_acceleration.z;

}

void viconCallback(const nav_msgs::Odometry::ConstPtr & vicon){
	vicon_vx = vicon->twist.twist.linear.x;
	vicon_vy = vicon->twist.twist.linear.y;
	vicon_vz = vicon->twist.twist.linear.z;
	q.w() = vicon->pose.pose.orientation.w;
	q.x() = vicon->pose.pose.orientation.x;
	q.y() = vicon->pose.pose.orientation.y;
	q.z() = vicon->pose.pose.orientation.z;
	vicon_z = vicon->pose.pose.position.z;
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
	ros::Subscriber sensor_sub=n.subscribe("opt_flow",10,sensorCallback);
	ros::Subscriber vicon_sub=n.subscribe("odom",10,viconCallback);
	ros::Subscriber imu_sub=n.subscribe("imu",10,imuCallback);
	filter_pub = n.advertise<opticalflow_msgs::OpticalSensor>("px4flow_data",10);
	ros::spin();
	return 0;
}
