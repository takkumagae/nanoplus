#include <ros/ros.h>
#include "adns3080.h"
#include <fcntl.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "adns3080_vis");

  ros::NodeHandle nh("~");

  std::string device;
  double output_rate;
  nh.param("device", device, std::string("/dev/spidev1.1"));
  nh.param("output_rate", output_rate, 10.0);

  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("image", 2);

  int fd = adns3080_init(device.c_str());
  if(fd == -1)
    return -1;

  uint8_t frame[900];
  sensor_msgs::Image img;
  img.height = 30;
  img.width = 30;
  img.encoding = sensor_msgs::image_encodings::MONO8;
  img.step = 30;
  img.data.resize(900);

  ros::Rate r(output_rate);
  while(nh.ok())
  {
    ros::spinOnce();
    if(adns3080_read_frame(fd, frame) != -1)
    {
      memcpy(&(img.data[0]), frame, 900);
      img_pub.publish(img);
    }
    r.sleep();
  }

  close(fd);
  return 0;
}
