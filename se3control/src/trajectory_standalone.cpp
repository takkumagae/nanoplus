#include <ros/ros.h>
#include <opticalflow_msgs/OpticalFlowCommand.h>
#include <Eigen/Geometry>
#include <quadrotor_msgs/OutputData.h>
#include <opticalflow_msgs/Traj.h>

int mode=1;
float des_t=0,des_vx=0,des_vy=0;
float rc_vx,rc_vy,rc_z;
double init_t=0;

void rcCallback(const quadrotor_msgs::OutputData::ConstPtr& rc)
{
	rc_vx = (float) -(rc->radio_channel[0]-137)/32;
	rc_vy = (float)  -(rc->radio_channel[1]-126)/32;
	rc_z = (float) rc->radio_channel[2]/128;
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
		ROS_WARN("Mannual Command: [des_vx,des_vy]: [%f,%f] ",des_vx,des_vy);
	}
	else if(mode==2)
	{
		ROS_WARN("Auto Command: [des_vx,des_vy,time]: [%f,%f,%f] ",des_vx,des_vy,des_t);
	}
	else if(mode==3)
	{
		ROS_WARN("Landing...");
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"trajectory");
	ros::NodeHandle n("~");
	ros::Publisher traj_pub=n.advertise<opticalflow_msgs::Traj>("trajectory",10);
	ros::Subscriber cmd_sub=n.subscribe("cmd",10,cmdCallback);
	ros::Subscriber rc_sub=n.subscribe("rc",10,rcCallback);
	ros::Rate loop_rate(100);
	opticalflow_msgs::Traj traj;
	Eigen::Vector3f vel,acc,pos;
	vel = Eigen::Vector3f::Zero(); 
	acc = Eigen::Vector3f::Zero(); 
	pos = Eigen::Vector3f::Zero(); 

	while(n.ok())
	{
		if(mode==1)
		{
			vel[0] = rc_vx;
			vel[1] = rc_vy;
			vel[2] = 0;
			pos[2] = rc_z;
		}
		else if(mode==2)
		{
			double cur_t = ros::Time::now().toSec();
			double t = cur_t-init_t;
			if(t<des_t){
				vel[0]=des_vx;
				vel[1]=des_vy;
				vel[2]=0;
				acc = Eigen::Vector3f::Zero();
			}
			else if(t>=des_t){
				vel = Eigen::Vector3f::Zero();
				acc = Eigen::Vector3f::Zero();
			}


		}

		traj.x = 0;
		traj.y = 0;
		traj.z = pos[2];
		traj.vx = vel[0];
		traj.vy = vel[1];
		traj.vz = vel[2];
		traj.acc_x = acc[0];
		traj.acc_y = acc[1];
		traj.acc_z = acc[2];
		traj.stop = 0;
		traj.mode = mode;
		traj.use_sensor = true;
		traj_pub.publish(traj);
		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
