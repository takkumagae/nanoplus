#include <ros/ros.h>
#include <std_msgs/String.h>
#include <quadrotor_msgs/SO3Command.h>
#include <opticalflow_msgs/Traj.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <opticalflow_msgs/StateMsg.h>
#include <opticalflow_msgs/OpticalSensor.h>
#include <math.h>

Eigen::Vector3f pos_des,vel_des,acc_des;
Eigen::Vector3f pos,vel;
//Eigen::Quaterniond q;
Eigen::Vector3f e1,e2,e3;
Eigen::Vector3f force,g;
Eigen::Matrix3f R_des;
Eigen::Vector3f b1w,b1,b2,b3;
static float KXY = 3, KVXY = 1.5, KZ = 3, KVZ = 2;
float k_xy = KXY, k_vxy = KVXY;
float k_z = KZ, k_vz = KVZ;
float k_R = 1,k_omg = 0.1;
static float m = 0.64; 
float of_vx,of_vy,of_vz,of_z;
static float offset_x = -0.5;
static float offset_y = 0.2;
ros::Publisher control_pub;
int mode=0;
bool use_sensor = false;
float stop=0,norm_force;
int pd_control;

void trajCallback(const opticalflow_msgs::Traj::ConstPtr & msg)
{

  pos_des[0] = msg->x;
  pos_des[1] = msg->y;
  pos_des[2] = msg->z;
  vel_des[0] = msg->vx;
  vel_des[1] = msg->vy;
  vel_des[2] = msg->vz;
  acc_des[0] = msg->acc_x;
  acc_des[1] = msg->acc_y;
  acc_des[2] = msg->acc_z;
  stop = msg->stop;
  mode = msg->mode;
	use_sensor = msg->use_sensor;
	if(stop==1)
	{
		k_xy =0;
		k_vxy = 0;
		k_z = 0;
		k_vz = 0;
	}
	else if(stop==0)
	{		
		k_xy = KXY;
		k_vxy = KVXY;
		k_z = KZ;
		k_vz = KVZ;
	}
}


void stateCallback(const nav_msgs::Odometry::ConstPtr& state)
{
  
	
		vel[0] = state->twist.twist.linear.x;
		vel[1] = state->twist.twist.linear.y;
		vel[2] = state->twist.twist.linear.z;
		pos[2] = state->pose.pose.position.z;
    
		//q.x() = state->pose.pose.orientation.x;
		//q.y() = state->pose.pose.orientation.y;
		//q.z() = state->pose.pose.orientation.z;
		//q.w() = state->pose.pose.orientation.w; 
    float q_x = state->pose.pose.orientation.x;
		float q_y = state->pose.pose.orientation.y;
		float q_z = state->pose.pose.orientation.z;
		float q_w = state->pose.pose.orientation.w;

float  yaw = atan2(2*(q_w*q_z+q_x*q_y),1-2*(q_y*q_y+q_z*q_z));
//Eigen::Matrix3d R(q);
	force(0) = offset_x+k_vxy*(vel_des(0)-vel(0))+m*acc_des(0);
	if(pd_control==0)
  force(1) = offset_y+k_vxy*(vel_des(1)-vel(1))+m*acc_des(1);
  else if(pd_control==1)
  {
    pos[1]=state->pose.pose.position.y;
    force(1) = offset_y+k_vxy*(vel_des(1)-vel(1))+k_xy*(0-pos[1])+m*acc_des(1);
  }
	force(2) = (k_z*(pos_des(2)-pos(2))+k_vz*(vel_des(2)-vel(2))+m*g(2)+m*acc_des(2));


  b3 = force.normalized();
  b2 = b3.cross(b1w);
  b1 = b2.cross(b3);
  R_des<<b1[0],b2[0],b3[0],
    b1[1],b2[1],b3[1],
    b1[2],b2[2],b3[2];
  Eigen::Quaternionf q_des(R_des);
  	
  quadrotor_msgs::SO3Command command;
	command.force.x = force[0];
  command.force.y = force[1];
  command.force.z = force[2];
  command.orientation.x = q_des.x();
  command.orientation.y = q_des.y();
  command.orientation.z = q_des.z();
  command.orientation.w = q_des.w();
  command.kR[0] = k_R;
  command.kR[1] = k_R;
  command.kR[2] = k_R;
  command.kOm[0] = k_omg;
  command.kOm[1] = k_omg;
  command.kOm[2] = k_omg;
	command.aux.current_yaw = yaw;
	command.aux.enable_motors = true;
	command.aux.use_external_yaw = true;
  control_pub.publish(command);
}

int main(int argc, char **argv)
{
  g<<0,
    0,
    9.81;
  b1w<<1,
    0,
    0;

ros::init(argc,argv,"se3control");
ros::NodeHandle nh("~");
nh.param("pd_control",pd_control,0);
ros::Subscriber trajectory_sub = nh.subscribe("trajectory",10,trajCallback);
ros::Subscriber state_sub = nh.subscribe("odom",10,stateCallback);
control_pub = nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd",10);

ros::spin();
return 0;
}

