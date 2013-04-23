#include <ros/ros.h>
#include <std_msgs/String.h>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_msgs/OutputData.h>
#include <opticalflow_msgs/OpticalFlowCommand.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <opticalflow_msgs/OpticalSensor.h>
#include <opticalflow_msgs/Traj.h>
#include <sonar/Range.h>

#define PI 3.1415926


Eigen::Vector3f pos_des,pos_des_b,vel_des,vel_des_b,acc_des,acc_des_b,pos,vel,pos_b,vel_b;
Eigen::Vector3f force,force_b,g,g_b;
Eigen::Matrix3f Rbw,R45;
float yaw_des=PI/4, roll_des, pitch_des;
static float KXY = 3, KVXY = 1.5, KZ = 2, KVZ = 1.5;
float k_xy = KXY, k_vxy = KVXY;
float k_z = KZ, k_vz = KVZ;
static float m = 0.26; 
ros::Publisher control_pub;
ros::Publisher traj_pub;
int mode=1,stop=0,pd_control;
bool use_sensor = false;
double init_t, init_stop, des_t = 0;
float des_vx =0, des_vy = 0;
float rc_vx=0,rc_vy=0,rc_z=0,rc_dyaw = 0;
float ax = 0, ay=0, a = 1.5;
float a3, a_max = 6, t1, t3 , des_s = 0.5;
double yaw_ctime = 0, yaw_ptime=0;

void rcCallback(const quadrotor_msgs::OutputData::ConstPtr& rc)
{
  rc_vx = (float) (rc->radio_channel[2]-122)/32;
  rc_vy = (float)  (rc->radio_channel[1]-126)/32;
//  rc_dyaw = (float) (rc->radio_channel[3]-127)/16;
	rc_z = (float) (rc->radio_channel[0]-34)/64;
//	yaw_ptime = yaw_ctime;
//	yaw_ctime = ros::Time::now().toSec();
//	yaw_des = yaw_des+(yaw_ctime-yaw_ptime)*rc_dyaw;
}

void cmdCallback(const opticalflow_msgs::OpticalFlowCommand::ConstPtr& msg)
{
  mode = msg->mode;
  des_vx = msg->vx;
  des_vy = msg->vy;
  des_t = msg->t;
  init_t = ros::Time::now().toSec();
  if(mode==1)
    ROS_WARN("Mannual Command:");
  else if(mode==2)
  {
		float v_max=sqrt(des_vx*des_vx+des_vy*des_vy);
		t1 = v_max/a;
		if(des_t>2*t1)
		{
			ROS_WARN("Auto Command:[des_vx: %f, des_vy: %f, des_t: %f]",des_vx,des_vy,des_t);
			ax = des_vx/v_max*a;
			ay = des_vy/v_max*a;
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
}

void irCallback(const sonar::Range::ConstPtr& msg)
{
	float ir = msg->range;
	if(vel_des[0]>0.5&&ir>0.5&&stop==0)
	{
		stop=1;
		init_stop = ros::Time::now().toSec();
    a3 = vel_des[0]*vel_des[0]/2.0/des_s;
		if(a3>a_max){
			ROS_WARN("DANGER, a3: [%f]",a3);
			a3=a_max;
		}
		t3 = vel_des[0]/a3;
		ROS_WARN("emergancy stop, a3: [%f], t3: [%f]",a3,t3);
	}
}

void stateCallback(const nav_msgs::Odometry::ConstPtr& state)
{

	pos_des[0]=0;
	pos_des[1]=0;
	pos_des[2]=rc_z;
	if(mode==1&&stop==0){
		vel_des[0]=rc_vx;
		vel_des[1]=rc_vy;
		vel_des[2]=0;
	}
	else if(mode==2&&stop==0)
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
			stop=1;
			init_stop = ros::Time::now().toSec();
			a3 = vel_des[0]*vel_des[0]/2.0/des_s;
			if(a3>a_max){
				ROS_WARN("DANGER, a3: [%f]",a3);
				a3=a_max;
			}
			t3 = vel_des[0]/a3;
			ROS_WARN("emergancy stop, a3: [%f], t3: [%f]",a3,t3);
			//		vel_des[0] = des_vx-ax*(t+t1-des_t);
			//vel_des[1] = des_vy-ay*(t+t1-des_t);
			//acc_des[0] = -ax;
			//acc_des[1] = -ay;
			//acc_des[2] = 0;
		}
		else if(t>=des_t){
			stop = 2;
			vel_des = Eigen::Vector3f::Zero();
			acc_des = Eigen::Vector3f::Zero();
		}
		vel_des[0] = vel_des[0]+rc_vx;
		vel_des[1] = vel_des[1]+rc_vy;
	}

	else if(stop==1)
	{
		acc_des[0]=-a3;
		double end_stop = ros::Time::now().toSec();
		double dt_stop = end_stop - init_stop;
		yaw_des = PI/4+(-PI/2)*dt_stop/t3;
		if(dt_stop>=t3)
		{
			stop=2;
			yaw_des = -PI/4;
			vel_des = Eigen::Vector3f::Zero();
			acc_des = Eigen::Vector3f::Zero();
		}
	}
  
	else if(stop==2)
	{
		pd_control = 0;
		vel_des = Eigen::Vector3f::Zero();
		acc_des = Eigen::Vector3f::Zero();
		vel_des[0] = vel_des[0]+rc_vx;
		vel_des[1] = vel_des[1]+rc_vy;
	}

  Eigen::Quaternionf q;
	q.x() = state->pose.pose.orientation.x;
	q.y() = state->pose.pose.orientation.y;
	q.z() = state->pose.pose.orientation.z;
	q.w() = state->pose.pose.orientation.w; 
  Eigen::Matrix3f Rbw(q);
  Eigen::Matrix3f Rcw = Rbw*R45.transpose();

	vel(0) = state->twist.twist.linear.x;
	vel(1) = state->twist.twist.linear.y;
	vel(2) = state->twist.twist.linear.z;
	pos_b(0) = state->pose.pose.position.x;
	pos_b(1) = state->pose.pose.position.y;
	pos_b(2) = state->pose.pose.position.z;
  
	Eigen::Matrix3f Rbw_des;
	Rbw_des<< cos(yaw_des-PI/4),-sin(yaw_des-PI/4),0,
		        sin(yaw_des-PI/4),cos(yaw_des-PI/4),0,
						0,0,1;
	pos = Rbw_des*pos_b;
//  ROS_INFO("pos:[%f,%f,%f]",pos[0],pos[1],pos[2]);	
	
	if(stop==0||stop==2){
		if(pd_control==0)
		{	
			force(0) = k_vxy*(vel_des(0)-vel(0))+m*(g(0)+acc_des(0));
			force(1) = k_vxy*(vel_des(1)-vel(1))+m*(g(1)+acc_des(1));
		}
			else if(pd_control==1)
		{
			force(0) = k_vxy*(vel_des(0)-vel(0))-k_xy*pos(0)+m*(g(0)+acc_des(0));
			force(1) = k_vxy*(vel_des(1)-vel(1))-k_xy*pos(1)+m*(g(1)+acc_des(1));
		}
		force(2) = k_z*(pos_des(2)-pos(2))+k_vz*(vel_des(2)-vel(2))+m*(g(2)+acc_des(2));
	}
	else if(stop==1)
	{
		force(0) = m*(g(0)+acc_des(0));
		force(1) = m*(g(1)+acc_des(1));
  }
//  ROS_INFO("force_b:[%f,%f,%f]",force_b(0),force_b(1),force_b(2));
//	force = Rbw*force_b; 
  force_b = Rbw.inverse()*force;	
  roll_des = (force(0)/m*sin(yaw_des)-force(1)/m*cos(yaw_des))/9.81;
  pitch_des = (force(0)/m*cos(yaw_des)+force(1)/m*sin(yaw_des))/9.81;  

	quadrotor_msgs::TRPYCommand command;
	command.thrust = force_b(2);
  command.roll = roll_des;
  command.pitch = pitch_des;
	command.yaw = yaw_des-PI/4;
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
R45<<sqrt(2)/2,-sqrt(2)/2,0,
	   sqrt(2)/2,sqrt(2)/2,0,
		 0,0,1;

pos_des = Eigen::Vector3f::Zero();
vel_des = Eigen::Vector3f::Zero();
acc_des = Eigen::Vector3f::Zero();
ros::init(argc,argv,"se3control");
ros::NodeHandle nh("~");
nh.param("pd_control",pd_control,0);
ros::Subscriber state_sub = nh.subscribe("odom",10,stateCallback);
ros::Subscriber cmd_sub=nh.subscribe("cmd",10,cmdCallback);
ros::Subscriber rc_sub=nh.subscribe("rc",10,rcCallback);
ros::Subscriber ir_sub=nh.subscribe("ir",10,irCallback);
control_pub = nh.advertise<quadrotor_msgs::TRPYCommand>("trpy_cmd",10);
traj_pub = nh.advertise<opticalflow_msgs::Traj>("trajectory",10);
ros::spin();
return 0;
}

