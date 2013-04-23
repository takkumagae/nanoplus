#include <ros/ros.h>
#include <quadrotor_msgs/Traj.h>
#include <Eigen/Geometry>
#include <quadrotor_msgs/Goal.h>

double reach = 0,mode=0,initial=0,init_t,des_v;
Eigen::Vector3d pos_f,pos_i,pos_f1,pos_f2;
Eigen::Vector3d dir;
Eigen::Vector3d vel_i,vel_f;
double dis1,dis2,dis3,des_t1,des_t2,des_t3,r;

void goalCallback(const quadrotor_msgs::Goal::ConstPtr& msg)
{
  pos_f[0] = msg->x;
  pos_f[1] = msg->y;
  pos_f[2] = msg->z;
  mode = msg->mode;
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
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"trajectory");
  ros::NodeHandle n;
  ros::Publisher traj_pub=n.advertise<quadrotor_msgs::Traj>("trajectory",1);
  ros::Subscriber goal_sub=n.subscribe("/mode",1,goalCallback);
  ros::Rate loop_rate(100);
  quadrotor_msgs::Traj traj;
  Eigen::Vector3d pos,vel,acc;
  pos_i[0] = 0;
  pos_i[1] = 0;
  pos_i[2] = 0;
  pos_f = pos_i;
  pos = pos_i;
  vel = pos;
  acc = vel;
  double des_t=1;
  double dis=0;
  while(n.ok())
  {

     if(mode==1)
    { 
      double cur_t = ros::Time::now().toSec();
      double t = cur_t-init_t;
      
      if(initial==0)
      {
        dis = sqrt((pos_f[0]-pos_i[0])*(pos_f[0]-pos_i[0])+(pos_f[1]-pos_i[1])*(pos_f[1]-pos_i[1])+(pos_f[2]-pos_i[2])*(pos_f[2]-pos_i[2]));
        des_t = dis/des_v;
        initial = 1;
        ROS_INFO("pos_i:[%f,%f,%f]",pos_i[0],pos_i[1],pos_i[2]);
      ROS_INFO("pos_f:[%f,%f,%f]",pos_f[0],pos_f[1],pos_f[2]);
      }
        
      r = t/des_t;
      if(r < 1){
        pos = pos_i+(pos_f-pos_i)*(10*r*r*r-15*r*r*r*r+6*r*r*r*r*r);
        vel = (pos_f-pos_i)*(30*r*r/des_t-60*r*r*r/des_t+30*r*r*r*r/des_t);
        acc = (pos_f-pos_i)*(60*r/des_t/des_t-180*r*r/des_t/des_t+120*r*r*r/des_t/des_t);
      }
      else if(r>1){
        pos = pos_f;
        vel = pos_f*0;
        acc = vel;
        if (reach==0){
          reach = 1;
          pos_i = pos_f;
          ROS_INFO("pos:[%f,%f,%f]",pos[0],pos[1],pos[2]);
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
        dis1 = 1;
        des_t1 = dis1/(des_v);
        pos_f1 = pos_i+dir*dis1;
        dis3 = 1;
        des_t3 = dis3/(des_v);
        dis2 = dis-dis1-dis3;
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
        r = t/des_t1;
        pos = pos_i+(pos_f1-pos_i)*(10*r*r*r-15*r*r*r*r+6*r*r*r*r*r)+(vel_f-vel_i)*(-4*r*r*t+7*r*r*r*t-3*r*r*r*r*t);
        vel = (pos_f1-pos_i)*(30*r*r/des_t1-60*r*r*r/des_t1+30*r*r*r*r/des_t1)+(vel_f-vel_i)*(-12*r*r+28*r*r*r-15*r*r*r*r);
        acc = (pos_f1-pos_i)*(60*r/des_t1/des_t1-180*r*r/des_t1/des_t1+120*r*r*r/des_t1/des_t1)+(vel_f-vel_i)*(-24*r/des_t1+84*r*r/des_t1-60*r*r*r/des_t1);
        
      }
      else if(t>=des_t1&&t<(des_t1+des_t2))
      {
        pos = pos_f1+vel_f*(t-des_t1);
        vel = vel_f;
        acc = vel_f*0;
      }

      else if(t>=(des_t1+des_t2)&&t<(des_t1+des_t2+des_t3))
      {
        if(reach ==0){
          reach = 1;
          pos_i = pos_f2;
        }
       t = t-des_t1-des_t2;
       t = (des_t3-t);
       r = t/des_t3;
   pos = pos_f+(pos_i-pos_f)*(10*r*r*r-15*r*r*r*r+6*r*r*r*r*r)+(vel_i-vel_f)*(-4*r*r*t+7*r*r*r*t-3*r*r*r*r*t);
        vel = (pos_f-pos_i)*(30*r*r/des_t3-60*r*r*r/des_t3+30*r*r*r*r/des_t3)+(vel_f-vel_i)*(-12*r*r+28*r*r*r-15*r*r*r*r);
        acc = (pos_i-pos_f)*(60*r/des_t3/des_t3-180*r*r/des_t3/des_t3+120*r*r*r/des_t3/des_t3)+(vel_i-vel_f)*(-24*r/des_t3+84*r*r/des_t3-60*r*r*r/des_t3);
        
      }


      if(t >= des_t1+des_t2+des_t3){
        pos = pos_f;
        vel = pos*0;
        acc = pos*0;
        ROS_INFO("pos:[%f,%f,%f]",pos[0],pos[1],pos[2]);
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

        traj_pub.publish(traj);
     ros::spinOnce();

     loop_rate.sleep();
  }
  return 0;
}
