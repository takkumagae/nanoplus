#include <ros/ros.h>
#include <opticalflow_msgs/Traj.h>
#include <Eigen/Geometry>
#include <opticalflow_msgs/Goal.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/OutputData.h>


double reach = 0,mode=0,initial=0,init_t,des_v;
Eigen::Vector3d pos_f,pos_i,pos_f1,pos_f2;
Eigen::Vector3d dir,a_vec,a_vec3;
Eigen::Vector3d vel_i,vel_f;
double dis1,dis2,dis3,des_t1,des_t2,des_t3,r,des_a = 4,des_a3 = 10;
double stop = 0;
bool use_sensor = false;
double bias_t =0.05,bias_dis = 0.0, stop_t = 0.0;
bool initial_vicon=false,initial_land=false;
double x_init=0,y_init=0,z_init=0;
double rc_vx,rc_vy,rc_z;

void viconCallback(const nav_msgs::Odometry::ConstPtr& vicon)
{
if(!initial_vicon)
{
	x_init = vicon->pose.pose.position.x;
	y_init = vicon->pose.pose.position.y;
	z_init = vicon->pose.pose.position.z;
  initial_vicon = true;
}
}

void rcCallback(const quadrotor_msgs::OutputData::ConstPtr& rc)
{
  rc_vx = (double) -(rc->radio_channel[0]-137)/128;
	rc_vy = (double)  -(rc->radio_channel[1]-126)/128;
	rc_z = (double) rc->radio_channel[2]/128;
}
void goalCallback(const opticalflow_msgs::Goal::ConstPtr& msg)
{
  pos_f[0] = msg->x;
  pos_f[1] = msg->y;
  pos_f[2] = msg->z;
  mode = msg->mode;
  use_sensor = msg->use_sensor;
	des_v = msg->v;
  init_t = ros::Time::now().toSec();
 	initial = 0;
  reach = 0;
  if(mode==1)
  {
    ROS_WARN("Minimum jerk...des_v:[%f],des_pos:[%f,%f,%f]",des_v,pos_f[0],pos_f[1],pos_f[2]);
  }
  else if(mode==2)
  {
    ROS_WARN("High speed...");
  }
	else if(mode==3)
	{
   ROS_WARN("Landing...");
	 initial_vicon=false;
	 initial_land =true;
	}
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"trajectory");
	ros::NodeHandle n;
	ros::Publisher traj_pub=n.advertise<opticalflow_msgs::Traj>("trajectory",10);
  ros::Subscriber goal_sub=n.subscribe("/mode",10,goalCallback);
	ros::Subscriber vicon_sub=n.subscribe("/odom",10,viconCallback);
	ros::Subscriber rc_sub=n.subscribe("/rc",10,rcCallback);
	ros::Rate loop_rate(100);
	opticalflow_msgs::Traj traj;
	Eigen::Vector3d pos,vel, acc;
	double des_t=1;
	double dis=0;
	while(n.ok())
	{
		if(mode==0){
			if(initial_vicon){
				pos_i[0] = x_init;
				pos_i[1] = y_init;
				pos_i[2] = z_init+0.1;
				pos_f = pos_i;

				pos = pos_i;
				vel = Eigen::Vector3d::Zero();
				acc = Eigen::Vector3d::Zero();

			}
		}
		else if(mode==4)
		{
		 vel[0] = rc_vx;
	   vel[1] = rc_vy;
		 vel[2] = 0;
		 pos[2] = rc_z;
	   acc = Eigen::Vector3d::Zero();	 
		}
		else if(mode==3)
		{
			if(initial_vicon&&initial_land){
				double cur_t = ros::Time::now().toSec();
				double t = cur_t-init_t;

				if(initial==0){
					pos_i[0] = x_init;
					pos_i[1] = y_init;
					pos_i[2] = z_init;
					pos_f[0] = x_init;
					pos_f[1] = y_init;
					pos_f[2] = 0;
					des_v = 0.2;
					dis = sqrt((pos_f[0]-pos_i[0])*(pos_f[0]-pos_i[0])+(pos_f[1]-pos_i[1])*(pos_f[1]-pos_i[1])+(pos_f[2]-pos_i[2])*(pos_f[2]-pos_i[2]));
					des_t = dis/des_v;
					ROS_INFO("pos_i:[%f,%f,%f]",pos_i[0],pos_i[1],pos_i[2]);
					ROS_INFO("pos_f:[%f,%f,%f]",pos_f[0],pos_f[1],pos_f[2]);

					initial=1;
				}
				if(dis>0)
					r = t/des_t;
				else if(dis==0)
					r=1;
				if(r < 1){
					pos = pos_i+(pos_f-pos_i)*(10*r*r*r-15*r*r*r*r+6*r*r*r*r*r);
					vel = (pos_f-pos_i)*(30*r*r/des_t-60*r*r*r/des_t+30*r*r*r*r/des_t);
					acc = (pos_f-pos_i)*(60*r/des_t/des_t-180*r*r/des_t/des_t+120*r*r*r/des_t/des_t);
				}
				else if(r>=1){
					pos = pos_f;
					vel = pos_f*0;
					acc = vel;
					if (reach==0){
						reach = 1;
						pos_i = pos_f;
					}
				}

			}
		}
			else if(mode==1)
    { 
      double cur_t = ros::Time::now().toSec();
      double t = cur_t-init_t;
      
      if(initial==0)
      {
				dis = sqrt((pos_f[0]-pos_i[0])*(pos_f[0]-pos_i[0])+(pos_f[1]-pos_i[1])*(pos_f[1]-pos_i[1])+(pos_f[2]-pos_i[2])*(pos_f[2]-pos_i[2]));
				des_t = dis/des_v;
				initial = 1;
				stop = 0;
				ROS_INFO("pos_i:[%f,%f,%f]",pos_i[0],pos_i[1],pos_i[2]);
				ROS_INFO("pos_f:[%f,%f,%f]",pos_f[0],pos_f[1],pos_f[2]);
      }
      if(dis>0)
				r = t/des_t;
			else if(dis==0)
				r=1;
      if(r < 1){
        pos = pos_i+(pos_f-pos_i)*(10*r*r*r-15*r*r*r*r+6*r*r*r*r*r);
        vel = (pos_f-pos_i)*(30*r*r/des_t-60*r*r*r/des_t+30*r*r*r*r/des_t);
        acc = (pos_f-pos_i)*(60*r/des_t/des_t-180*r*r/des_t/des_t+120*r*r*r/des_t/des_t);
      }
      else if(r>=1){
        pos = pos_f;
        vel = pos_f*0;
        acc = vel;
        if (reach==0){
          reach = 1;
          pos_i = pos_f;
        }
        }
    }
     else  if(mode==2)
    { 
      double cur_t = ros::Time::now().toSec();
      double t = cur_t-init_t;
      if(initial==0)
      {
        dis = sqrt((pos_f[0]-pos_i[0])*(pos_f[0]-pos_i[0])+(pos_f[1]-pos_i[1])*(pos_f[1]-pos_i[1])+(pos_f[2]-pos_i[2])*(pos_f[2]-pos_i[2]));
        dir = (pos_f-pos_i)/dis;
        a_vec = dir*des_a;
        dis1 = des_v*des_v/(2*des_a);
        des_t1 = des_v/des_a;
				pos_f1 = pos_i+dir*dis1;
				a_vec3 = dir*des_a3;	
        des_t3 = des_v/des_a3;
        dis3 = des_v*des_v/(2*des_a3);
        dis2 = dis-dis1-dis3-bias_dis;
        pos_f2 = pos_f1+dir*dis2;
        des_t2 = dis2/des_v;
        vel_f = dir*des_v;
        vel_i = vel_f*0;
        initial = 1;
        ROS_INFO("pos_i:[%f,%f,%f]",pos_i[0],pos_i[1],pos_i[2]);
        ROS_INFO("pos_f1:[%f,%f,%f]",pos_f1[0],pos_f1[1],pos_f1[2]);
        ROS_INFO("pos_f2:[%f,%f,%f]",pos_f2[0],pos_f2[1],pos_f2[2]);
        ROS_INFO("STATE: des_t:[%f,%f,%f]",des_t1,des_t2,des_t3);
      }
        
      if(t<des_t1)
      { 
        
        pos = pos_i+0.5*a_vec*t*t;
        vel = vel_i+a_vec*t;
        acc = a_vec;
        stop = 0;
      }
      else if(t>=des_t1&&t<(des_t1+des_t2))
      {
        pos = pos_f1+vel_f*(t-des_t1);
        vel = vel_f;
        acc = vel_f*0;
				stop = 0;
      }

      else if(t>=(des_t1+des_t2)&&t<(des_t1+des_t2+des_t3+bias_t))
      {
				
				t = t-des_t1-des_t2;
				pos = pos_f2+vel_f*t-0.5*a_vec3*t*t;
				vel = vel_f-a_vec3*t;
				acc = -a_vec3;
        stop = 1;
	
			}

			else if(t >= des_t1+des_t2+des_t3+bias_t && t<des_t1+des_t2+des_t3+bias_t+stop_t){
        stop = 1;
				pos = pos_f;
        vel = pos*0;
        acc = -a_vec3*(1-(t-des_t1-des_t2-des_t3-bias_t)/stop_t);
        pos_i = pos_f; 
      }
			else if(t>des_t1+des_t2+des_t3+bias_t+stop_t)
			{
				stop = 0;
				pos = pos_f;
				vel = pos*0;
				acc = vel;
				pos_i = pos_f; 

			}
		}
		 traj.x = pos[0];
		 traj.y = pos[1];
		 traj.z = pos[2];
		 traj.vx = vel[0];
		 traj.vy = vel[1];
		 traj.vz = vel[2];
		 traj.acc_x = acc[0];
		 traj.acc_y = acc[1];
		 traj.acc_z = acc[2];
		 traj.stop = stop;
		 traj.mode = mode;
		 traj.use_sensor = use_sensor;
		 traj_pub.publish(traj);
		 ros::spinOnce();

		 loop_rate.sleep();
	}
  return 0;
}
