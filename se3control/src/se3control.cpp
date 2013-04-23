#include <ros/ros.h>
#include <std_msgs/String.h>
#include <quadrotor_msgs/SO3Command.h>
#include <opticalflow_msgs/Traj.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <opticalflow_msgs/StateMsg.h>
#include <opticalflow_msgs/OpticalSensor.h>


Eigen::Vector3d pos_des,vel_des,acc_des;
Eigen::Vector3d pos,vel;
Eigen::Quaterniond q;
Eigen::Vector3d e1,e2,e3;
Eigen::Vector3d force,g;
Eigen::Matrix3d R_des;
Eigen::Vector3d b1w,b1,b2,b3;
double k_xy = 3, k_vxy = 1.5;
double k_z = 3, k_vz = 2;
double k_R = 1,k_omg = 0.1;
double m = 0.56; 
double of_vx,of_vy,of_vz,of_z;
double offset_x = -0.25,offset_y = 0.13;
ros::Publisher control_pub;
ros::Publisher state_msg_pub;
int mode=0;
bool use_sensor = false;
double stop=0,norm_force;

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
		k_xy = 3;
		k_vxy = 1.5;
		k_z = 5;
		k_vz = 3;
	}
}

void opticalflowCallback(const nav_msgs::Odometry::ConstPtr& of)
{
  of_vx = of->twist.twist.linear.x;
	of_vy = of->twist.twist.linear.y;
	of_vz = of->twist.twist.linear.z;
	of_z = of->pose.pose.position.z;
}

void stateCallback(const nav_msgs::Odometry::ConstPtr& state)
{
  
	opticalflow_msgs::StateMsg state_msg;
  pos[0] = state->pose.pose.position.x;
  pos[1] = state->pose.pose.position.y;
	state_msg.vicon_vx = state->twist.twist.linear.x;
	state_msg.vicon_vy = state->twist.twist.linear.y;
  state_msg.sensor_vx = of_vx;
	state_msg.sensor_vy = of_vy;
	if(!use_sensor){
		vel[0] = state_msg.vicon_vx;
		vel[1] = state_msg.vicon_vy;
		vel[2] = state->twist.twist.linear.z;
		pos[2] = state->pose.pose.position.z;
	}
	else if(use_sensor){
		vel[0] = state_msg.vicon_vx;
		vel[1] = state_msg.vicon_vy;
		vel[2] = of_vz;
		pos[2] = of_z;
	}
  q.x() = state->pose.pose.orientation.x;
  q.y() = state->pose.pose.orientation.y;
  q.z() = state->pose.pose.orientation.z;
  q.w() = state->pose.pose.orientation.w; 
	
	state_msg.roll = atan2(2*(q.w()*q.x()+q.y()*q.z()),1-2*(q.x()*q.x()+q.y()*q.y()));
	state_msg.pitch = asin(2*(q.w()*q.y()-q.z()*q.x()));
 	state_msg.yaw = atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
  
	//Eigen::Matrix3d R(q);
  if(mode!=4) 
	{
		force(0) = offset_x+(k_xy*(pos_des(0)-pos(0))+k_vxy*(vel_des(0)-vel(0))+m*acc_des(0));
		force(1) = offset_y+(k_xy*(pos_des(1)-pos(1))+k_vxy*(vel_des(1)-vel(1))+m*acc_des(1));
		std::cout<<"offset_x = "<<offset_x-force(0)<<", offset_y = "<<offset_y-force(1)<<std::endl;
	}
	else if(mode==4)
	{
		force(0) = offset_x+k_vxy/3*(vel_des(0)-vel(0))+m*acc_des(0);
		force(1) = offset_y+k_vxy/3*(vel_des(1)-vel(1))+m*acc_des(1);
	}
  if(!use_sensor){
		force(2) = (k_z*(pos_des(2)-pos(2))+k_vz*(vel_des(2)-vel(2))+m*g(2)+m*acc_des(2));
	}
  else if(use_sensor){
	force(2) = (k_z*(pos_des(2)-pos(2))+k_vz*(vel_des(2)-vel(2))+m*g(2)+m*acc_des(2));
	}	


  b3 = force.normalized();
//  ROS_INFO("b3:[%f,%f,%f]",b3[0],b3[1],b3[2]);
  b2 = b3.cross(b1w);
  b1 = b2.cross(b3);
  R_des<<b1[0],b2[0],b3[0],
    b1[1],b2[1],b3[1],
    b1[2],b2[2],b3[2];
  Eigen::Quaterniond q_des(R_des);
  	
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
	command.aux.current_yaw = state_msg.yaw;
	command.aux.enable_motors = true;
	command.aux.use_external_yaw = true;
  control_pub.publish(command);

	state_msg.des_roll = atan2(2*(q_des.w()*q_des.x()+q_des.y()*q_des.z()),1-2*(q_des.x()*q_des.x()+q_des.y()*q_des.y()));
	state_msg.des_pitch = asin(2*(q_des.w()*q_des.y()-q_des.z()*q_des.x()));
	state_msg.des_yaw = atan2(2*(q_des.w()*q_des.z()+q_des.x()*q_des.y()),1-2*(q_des.y()*q_des.y()+q_des.z()*q_des.z()));
	state_msg.x = pos(0);
	state_msg.y = pos(1);
	state_msg.z = pos(2);
  state_msg.vx = vel(0);
	state_msg.vy = vel(1);
	state_msg.vz = vel(2);
  state_msg.des_x = pos_des(0);
	state_msg.des_y = pos_des(1);
	state_msg.des_z = pos_des(2);
	state_msg.des_vx = vel_des(0);
	state_msg.des_vy = vel_des(1);
	state_msg.des_vz = vel_des(2);
	state_msg.des_accx = acc_des(0);
	state_msg.des_accy = acc_des(1);
	state_msg.des_accz = acc_des(2);

  state_msg_pub.publish(state_msg);
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
ros::Subscriber trajectory_sub = nh.subscribe("/trajectory",10,trajCallback);
ros::Subscriber state_sub = nh.subscribe("odom",10,stateCallback);
ros::Subscriber opticalflow_sub = nh.subscribe("optical_flow",10,opticalflowCallback);
control_pub = nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd",10);
state_msg_pub = nh.advertise<opticalflow_msgs::StateMsg>("state_msg",10);

ros::spin();
return 0;
}

