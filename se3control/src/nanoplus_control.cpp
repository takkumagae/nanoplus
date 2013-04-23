#include <ros/ros.h>
#include <std_msgs/String.h>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_msgs/OutputData.h>
#include <opticalflow_msgs/OpticalFlowCommand.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <opticalflow_msgs/OpticalSensor.h>
#include <opticalflow_msgs/Traj.h>
#define PI 3.1415926


Eigen::Vector3f pos_des,vel_des,acc_des;
Eigen::Vector3f pos,vel;
Eigen::Vector3f force,force_b,g;
Eigen::Matrix3f Rbw;
float yaw_des, roll_des, pitch_des;
static float KXY = 3, KVXY = 1.5, KZ = 2, KVZ = 1.5;
float k_xy = KXY, k_vxy = KVXY;
float k_z = KZ, k_vz = KVZ;
static float m = 0.26; 
static float offset_x = 0;
static float offset_y = 0;
ros::Publisher control_pub;
ros::Publisher traj_pub;
int mode=1;
bool use_sensor = false;
int stop=0;
int pd_control;
double init_t, t1, des_t = 0;
float des_vx =0, des_vy = 0;
float rc_vx=0,rc_vy=0,rc_z=0;
float ax = 0, ay=0, a = 1.5;
float ax3 = 0, ay3 = 0, a_max = 6, t3;
void rcCallback(const quadrotor_msgs::OutputData::ConstPtr& rc)
{
  rc_vx = (float) (rc->radio_channel[2]-122)/32;
  rc_vy = (float)  (rc->radio_channel[1]-126)/32;
  rc_z = (float) (rc->radio_channel[0]-34)/64;
}

void cmdCallback(const opticalflow_msgs::OpticalFlowCommand::ConstPtr& msg)
{
  mode = msg->mode;
  des_vx = msg->vx;
  des_vy = msg->vy;
  des_t = msg->t;
  init_t = ros::Time::now().toSec();
  if(mode==1)
  {
    ROS_WARN("Mannual Command:");
  }
  else if(mode==2)
  {
		t1 = sqrt(des_vx*des_vx+des_vy*des_vy)/a;
		if(des_t>2*t1)
			{

       ROS_WARN("Auto Command:[des_vx: %f, des_vy: %f, des_t: %f]",des_vx,des_vy,des_t);
			 ax = des_vx/sqrt(des_vx*des_vx+des_vy*des_vy)*a;
			 ay = des_vy/sqrt(des_vx*des_vx+des_vy*des_vy)*a;
}
		else if(des_t<=2*t1){
			ROS_WARN("time step too small...");
			des_vx = 0;
			des_vy = 0;
			des_t = 0;
      ax=0;
      ay=0;
		}
}
else if(mode==3)
{
  float v_max = sqrt(des_vx*des_vx+des_vy*des_vy);
	t1 = v_max/a;
  float a3 = v_max/0.5;
  if(a3>a_max)
    a3 = a_max;
	t3 = v_max/a3;
	ROS_WARN("Instant mode... t3:[%f], a3:[%f]",t3,a3);
	ax = des_vx/sqrt(des_vx*des_vx+des_vy*des_vy)*a;
	ay = des_vy/sqrt(des_vx*des_vx+des_vy*des_vy)*a;
  ax3 = des_vx/sqrt(des_vx*des_vx+des_vy*des_vy)*a3;
	ay3 = des_vy/sqrt(des_vx*des_vx+des_vy*des_vy)*a3;
}
}

void stateCallback(const nav_msgs::Odometry::ConstPtr& state)
{

	if(mode==1){
		vel_des[0]=rc_vx;
		vel_des[1]=rc_vy;
		vel_des[2]=0;
		pos_des[1]=0;
		pos_des[2]=rc_z;
	}
	else if(mode==2)
	{
		double cur_t = ros::Time::now().toSec();
		double t = cur_t-init_t;
    
		vel_des[2]=0;
    
		if(t<t1){
			vel_des[0]=ax*t;
			vel_des[1]=ay*t;
      acc_des[0]=ax;
      acc_des[1]=ay;
      acc_des[2]=0;
		}
		else if(t<des_t-t1&&t>=t1){
			vel_des[0]=des_vx;
			vel_des[1]=des_vy;
			acc_des = Eigen::Vector3f::Zero();
	}
		else if(t>=des_t-t1&&t<des_t){
			vel_des[0] = des_vx-ax*(t+t1-des_t);
			vel_des[1] = des_vy-ay*(t+t1-des_t);
      acc_des[0] = -ax;
      acc_des[1] = -ay;
      acc_des[2] = 0;
		}
		else if(t>=des_t){
			vel_des = Eigen::Vector3f::Zero();
			acc_des = Eigen::Vector3f::Zero();
		}
		vel_des[0] = vel_des[0]+rc_vx/2;
		vel_des[1] = vel_des[1]+rc_vy/2;
	}

else if(mode==3)
	{
		double cur_t = ros::Time::now().toSec();
		double t = cur_t-init_t;
    
		vel_des[2]=0;
    
		if(t<t1){
			vel_des[0]=ax*t;
			vel_des[1]=ay*t;
      acc_des[0]=ax;
      acc_des[1]=ay;
      acc_des[2]=0;
		}
		else if(t<des_t-t3&&t>=t1){
			vel_des[0]=des_vx;
			vel_des[1]=des_vy;
			acc_des = Eigen::Vector3f::Zero();
		}
		else if(t>=des_t-t3&&t<des_t){
			vel_des[0] = 0;
			vel_des[1] = 0;
      acc_des[0] = -ax3;
      acc_des[1] = -ay3;
      acc_des[2] = 0;
      stop = 1;
		}
		else if(t>=des_t){
      stop = 0;
			vel_des = Eigen::Vector3f::Zero();
			acc_des = Eigen::Vector3f::Zero();
		}
		vel_des[0] = vel_des[0]+rc_vx/2;
		vel_des[1] = vel_des[1]+rc_vy/2;
	}

	vel(0) = state->twist.twist.linear.x;
	vel(1) = state->twist.twist.linear.y;
	vel(2) = state->twist.twist.linear.z;
	pos(2) = state->pose.pose.position.z;

  Eigen::Quaternionf q;
	q.x() = state->pose.pose.orientation.x;
	q.y() = state->pose.pose.orientation.y;
	q.z() = state->pose.pose.orientation.z;
	q.w() = state->pose.pose.orientation.w; 
  Eigen::Matrix3f R(q);
	if(stop==0){
		force(0) = offset_x+k_vxy*(vel_des(0)-vel(0))+m*acc_des(0);
		if(pd_control==0)
			force(1) = offset_y+k_vxy*(vel_des(1)-vel(1))+m*acc_des(1);
		else if(pd_control==1)
		{
			pos(1)=state->pose.pose.position.y;
			force(1) = offset_y+k_vxy*(vel_des(1)-vel(1))+k_xy*(0-pos(1))+m*acc_des(1);
		}
		force(2) = (k_z*(pos_des(2)-pos(2))+k_vz*(vel_des(2)-vel(2))+m*g(2)+m*acc_des(2));
	}
	else if(stop==1)
	{
		force(0) = offset_x+m*acc_des(0);
		force(1) = offset_y+m*acc_des(1);
  }


 force_b = R.transpose()*force; 	
 yaw_des = PI/4;
  roll_des = (force(0)/m*sin(yaw_des)-force(1)/m*cos(yaw_des))/9.81;
  pitch_des = (force(0)/m*cos(yaw_des)+force(1)/m*sin(yaw_des))/9.81;  

	quadrotor_msgs::TRPYCommand command;
	command.thrust = force_b(2);
  command.roll = roll_des;
  command.pitch = pitch_des;
	command.yaw = 0;
	command.aux.current_yaw = 0;
  command.aux.enable_motors = true;
  command.aux.use_external_yaw = true;
  
  control_pub.publish(command);

	opticalflow_msgs::Traj traj;

	traj.x = pos_des[0];
	traj.y = pos_des[1];
	traj.z = pos_des[2];
	traj.vx = vel_des[0];
	traj.vy = vel_des[1];
	traj.vz = vel_des[2];
	traj.acc_x = acc_des[0];
	traj.acc_y = acc_des[1];
	traj.acc_z = acc_des[2];
	traj.stop = 0;
	traj.mode = mode;
	traj.use_sensor = true;
	traj_pub.publish(traj);

}

int main(int argc, char **argv)
{
  g<<0,
    0,
    9.81;

pos_des = Eigen::Vector3f::Zero();
vel_des = Eigen::Vector3f::Zero();
acc_des = Eigen::Vector3f::Zero();
ros::init(argc,argv,"se3control");
ros::NodeHandle nh("~");
nh.param("pd_control",pd_control,0);
ros::Subscriber state_sub = nh.subscribe("odom",10,stateCallback);
ros::Subscriber cmd_sub=nh.subscribe("cmd",10,cmdCallback);
ros::Subscriber rc_sub=nh.subscribe("rc",10,rcCallback);
control_pub = nh.advertise<quadrotor_msgs::TRPYCommand>("trpy_cmd",10);
traj_pub = nh.advertise<opticalflow_msgs::Traj>("trajectory",10);
ros::spin();
return 0;
}

