#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "adns3080.h"
#include <fcntl.h>

class ADNS3080 : public nodelet::Nodelet
{
 public:
  ~ADNS3080(void);
  void onInit(void);

 private:
  void output_callback(const ros::TimerEvent &event);

  int fd_;
  bool connected_;
  ros::Publisher output_pub_;
  ros::Timer output_timer_;
};

void ADNS3080::onInit(void)
{
  ros::NodeHandle nh(getPrivateNodeHandle());

  std::string device;
  double output_rate;
  nh.param("device", device, std::string("/dev/spidev1.1"));
  nh.param("output_rate", output_rate, 10.0);

  output_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("optical_flow", 10);

  fd_ = adns3080_init(device.c_str());
  if(fd_ == -1)
  {
    connected_ = false;
    return;
  }

  connected_ = true;
  output_timer_ = nh.createTimer(ros::Duration(1/output_rate), &ADNS3080::output_callback, this);
}

void ADNS3080::output_callback(const ros::TimerEvent &event)
{
  geometry_msgs::Vector3Stamped::Ptr output_msg(new geometry_msgs::Vector3Stamped);
  adns3080_output output = adns3080_read_motion_burst(fd_);
  output_msg->header.stamp = ros::Time::now();
  if((output.motion & 0x80) == 0x80)
  {
    const float flow_x = output.delta_x;
    const float flow_y = output.delta_y;
    output_msg->vector.x = flow_y;//cosf(135*M_PI/180)*flow_x - sinf(135*M_PI/180)*flow_y;
    output_msg->vector.y = -flow_x;//sinf(135*M_PI/180)*flow_x + cosf(135*M_PI/180)*flow_y;
    output_msg->vector.z = output.squal;
  }
  else
    output_msg->vector.z = -10;
  output_pub_.publish(output_msg);
}


ADNS3080::~ADNS3080(void)
{
  if(connected_)
  {
    close(fd_);
    connected_ = false;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(adns3080, ADNS3080, ADNS3080, nodelet::Nodelet);
