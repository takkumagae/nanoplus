#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <opticalflow_msgs/OpticalFlow.h>
#include <quadrotor_msgs/OutputData.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sonar/Range.h>

Eigen::Vector2f state_x,state_y;
Eigen::Vector3f state_z;
Eigen::Matrix2f P_x,P_y,Q_x,Q_y,I;
Eigen::Matrix3f P_z,Q_z;
Eigen::MatrixXf H(1,2),HT(2,1),B_x(2,1),B_y(2,1);
Eigen::Vector2f K_x,K_y;
Eigen::Matrix2f A,AT;
Eigen::Matrix3f R45;
Eigen::Vector3f v,w_c,w_b,vel,pos;
Eigen::Vector3f acc,g,g_b;
Eigen::Quaternionf q,q_i;
float S_x,S_y,Y_x,Y_y;
double current = 0, previous=0, current_o = 0,previous_o=0,dt;
double R = 0.4,R_y,R_z;
float bias_vx,bias_vy;
ros::Publisher odom_pub;
bool init_q = false;
double init_sonar;
float dis_y=0,Py=0.01;
double Q_z1,Q_z2,Q_z3;
double Q_y1,Q_y2,Q_py;

void stateCallback(const quadrotor_msgs::OutputData::ConstPtr& msg)
{
  w_b[0] = msg->angular_velocity.x;
  w_b[1] = msg->angular_velocity.y;
  w_b[2] = msg->angular_velocity.z;
  
	acc <<  msg->linear_acceleration.x,
          msg->linear_acceleration.y,
          msg->linear_acceleration.z;

  q.w() = msg->orientation.w;
  q.x() = msg->orientation.x;
  q.y() = msg->orientation.y;
  q.z() = msg->orientation.z;
  if(!init_q){
    init_q = true;
    q_i.w() = q.w();
    q_i.x() = q.x();
    q_i.y() = q.y();
    q_i.z() = q.z();
    q_i = q_i.inverse();
  }
  
  q = q_i*q;
	Eigen::Matrix3f Rbw(q);
	Eigen::Matrix3f Rcw = Rbw*R45.transpose();
	w_c = R45*w_b;
// Eigen::Vector3f ypr = Rbw.eulerAngles(2,1,0);
//  yaw = ypr(0);
//  pitch = ypr(1);
//  roll = ypr(2);
  
  acc = R45*acc;
	
	previous = current;
  current = ros::Time::now().toSec();
  dt = current-previous;
  if(dt>10)
  dt = 1/100;
  A<<1,-dt,
    0,1;
  AT = A.transpose();
	g_b = Rcw.inverse()*g;
float	nbx = (drand48()-0.5)*0.1;
float	nby = (drand48()-0.5)*0.1;
	B_x<<dt*(acc(0)-g_b(0)),
	    dt*nbx;
	B_y<<dt*(acc(1)-g_b(1)),
		dt*nby;
	state_x = A*state_x+B_x;
	P_x = A*P_x*AT+Q_x;
	state_y = A*state_y+B_y;
	P_y = A*P_y*AT+Q_y;
  
  Eigen::Matrix3f Az; 
  Az<<1,dt,-0.5*dt*dt,
    0,1,-dt,
    0,0,1;
  Eigen::Vector3f Bz;
  float nbz=0.5*(drand48()-0.5);
  Bz << 0.5*dt*dt*(acc(2)-g_b(2)),
     dt*(acc(2)-g_b(2)),
     dt*nbz;


  state_z = Az*state_z+Bz;
  P_z = Az*P_z*Az.transpose()+Q_z;

  dis_y = dis_y+dt*state_y(0);
  Py = Py+Q_py;

  vel[0] = state_x[0];
	vel[1] = state_y[0];
	vel[2] = state_z[1];
	vel = Rbw*vel;
 
	pos[0] = 0;
	pos[1] = dis_y;
	pos[2] = state_z[0];
//	pos = Rbw*pos;

	nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "/map";
  odom.pose.pose.position.x = pos[0];
  odom.pose.pose.position.y = pos[1];
  odom.pose.pose.position.z = pos[2];
  odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();
	odom.twist.twist.linear.x = vel[0];
	odom.twist.twist.linear.y = vel[1];
  odom.twist.twist.linear.z = vel[2];
  odom.twist.twist.angular.x = w_b[0];
  odom.twist.twist.angular.y = w_b[1];
  odom.twist.twist.angular.z = w_b[2];

  odom_pub.publish(odom);
}
void sensorCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	bias_vx = w_c[1]*0.6;
	bias_vy = w_c[0]*0.6;
	v(0) = msg->vector.x/25*state_z(0);//+bias_vx;
	v(1) = msg->vector.y/25*state_z(0);//-bias_vy;
	v(2) = state_z(1);
//	v  = Rbw*v;
	previous_o = current_o;
	current_o = ros::Time::now().toSec();
	double dt_o = current_o-previous_o;

	if(dt_o>1)
		dt_o = 1/40;

	S_x = P_x(0,0)+R;
	K_x = P_x*HT/S_x;
	Y_x = v(0) - state_x(0);
	state_x = state_x+K_x*Y_x;
	P_x = (I-K_x*H)*P_x;

	S_y = P_y(0,0)+R;
	K_y = P_y*HT/S_y;
	Y_y = v(1) - state_y(0);
	state_y = state_y+K_y*Y_y;
	P_y = (I-K_y*H)*P_y;

}

void leftsonarCallback(const sonar::Range::ConstPtr& msg)
{
  float left_sonar = msg->range;

  if(left_sonar<3)
  {
    double Sy,Yy,Ky;
    Sy = Py+R_y;
    Ky = Py/Sy;
    Yy = (init_sonar-left_sonar)-dis_y;
    dis_y = dis_y+Ky*Yy;
    Py = (1-Ky)*Py;

  }

}
void downsonarCallback(const sonar::Range::ConstPtr& msg)
{
	float down_sonar = msg->range;
	Eigen::Vector3f Kz;
	Eigen::MatrixXf Hz(1,3);
	Eigen::Matrix3f Iz;
	Iz<<1,0,0,
		0,1,0,
		0,0,1;
	Hz<<1,0,0;
	if(down_sonar<3)
	{
		double Sz,Yz;
		Sz = P_z(0,0)+R_z;
		Kz = P_z*Hz.transpose()/Sz;
		Yz = down_sonar-state_z(0);
		state_z = state_z+Kz*Yz;
		P_z = (Iz-Kz*Hz)*P_z;
   }

}


int main(int argc, char **argv)
{
	g<<0,
		0,
		9.81;
  I<<1,0,
    0,1;
  H<<1,0;
  HT<<1,
    0;

  state_x<<0,
    0;
  state_y = state_x;
  P_x << 0.1,0,
      0,0.1;
  P_y = P_x;
  P_z << 0.1,0,0,
        0,0.1,0,
				0,0,0.1;
	R45<< sqrt(2)/2,-sqrt(2)/2,0,
        sqrt(2)/2,sqrt(2)/2,0,
        0,0,1;

  ros::init(argc,argv,"filter");
  ros::NodeHandle n("~");
  ROS_INFO("ready....");
	n.param("z/Q_z1",Q_z1,0.00004);
	n.param("z/Q_z2",Q_z2,0.0025);
	n.param("z/Q_z3",Q_z3,0.09);
  n.param("y/Q_y1",Q_y1,0.00001);
  n.param("y/Q_y2",Q_y2,0.0025);
  n.param("y/Q_py",Q_py,0.0001);
 	n.param("z/R_z",R_z,0.09);
	n.param("y/R_y",R_y,0.09); 
  n.param("init_sonar",init_sonar,0.55);
  
  Q_y << Q_y1,0,
         0,Q_y2;
  Q_x = Q_y;
  Q_z << Q_z1,0,0,
      0,Q_z2,0,
      0,0,Q_z3;
 
  ros::Subscriber sensor_sub=n.subscribe("optical_flow",10,sensorCallback);
  ros::Subscriber state_sub=n.subscribe("imu",10,stateCallback);
  ros::Subscriber leftsonar_sub=n.subscribe("left",10,leftsonarCallback);
  ros::Subscriber downsonar_sub=n.subscribe("down",10,downsonarCallback);
	odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
  ros::spin();
	return 0;
}
