#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <cstring>
#include <cstdlib>
#include <string.h>
#include <sys/utsname.h>

#include <ros/ros.h>
#include "sonar/Range.h"

typedef uint8_t u8;
typedef uint16_t u16;
#include "twl4030-madc.h"

//#define MADC_DEVICE_NEW "/sys/class/hwmon/hwmon0/device/in2_input"
#define MADC_DEVICE_OLD "/dev/twl4030-madc"

#define limit(x, a, b) ( (x) < (a) ? (a) : ( (x) > (b) ? (b) : (x) ) )

/* Channel numbering:
 * ADC0-1 : to do with battery charging, not relevant on Overo
 * ADC2-7 : general purpose, input range = 0 - 2.5V.
 * ADC8 : USB OTG port bus voltage.
 * ADC9-11 : more battery charging stuff, not relevant.
 * ADC12 : main battery voltage.
 *         This will be the system 3.3V rail in our case
 * ADC13-15: reserved or not relevant.
 */

bool check_kernel_version(int kernel_version, int major_version, int minor_version);


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "sonar");

  ros::NodeHandle n("~");

  int rate;
  n.param("rate", rate, 10);
  std::string MADC_DEVICE_NEW;
  n.param("channel",MADC_DEVICE_NEW,std::string("/sys/class/hwmon/hwmon0/device/in2_input"));
  int adc_channel;
  n.param("adc_channel", adc_channel, 2);

  std::string frame;
  n.param("frame", frame, std::string("sonar"));

  double min_range;
  n.param("min_range", min_range, 0.2);
  double max_range;
  n.param("max_range", max_range, 6.0);

  double mV_per_cm;
  n.param("mV_per_cm", mV_per_cm, (5.0/1024)*1000);

  const bool new_kernel = check_kernel_version(2,6,39);

  int d = -1;
  struct twl4030_madc_user_parms par;

  FILE *fp = NULL;
  if(new_kernel)
    {
      fp = fopen(MADC_DEVICE_NEW.c_str(), "r");
      if(fp == NULL)
        {
          ROS_FATAL("%s: could not open ADC device %s\n",
                    ros::this_node::getName().c_str(),
                    MADC_DEVICE_NEW.c_str());
          return -1;
        }
    }
  else
    {
      d = open(MADC_DEVICE_OLD, O_RDWR | O_NONBLOCK);
      if (d == -1)
        {
          ROS_FATAL("%s: could not open ADC device %s\n",
                    ros::this_node::getName().c_str(),
                    MADC_DEVICE_OLD);
          return -1;
        }

      memset(&par, 0, sizeof(struct twl4030_madc_user_parms));

      par.channel = adc_channel;
    }
  ros::Publisher rangePub = n.advertise<sonar::Range>("sonar_range", 10);

  sonar::Range rangeMsg;
  rangeMsg.header.seq = 0;
  rangeMsg.header.frame_id = frame;
  rangeMsg.min_range =min_range;
  rangeMsg.max_range = max_range;

  ros::Rate r(rate);

  int ret;
  float range = 0;
  while(n.ok())
    {
      if(new_kernel)
        {
          rewind(fp);
          char buf[6];
          if(fread(buf, 1, 6, fp) > 0)
            {
              int val = atoi(buf);
              range = val / mV_per_cm / 100;
              rangeMsg.header.stamp = ros::Time::now();
            }
          else
            {
              ROS_WARN("ADC read error\n");
            }
        }
      else
        {
          ret = ioctl(d, TWL4030_MADC_IOCX_ADC_RAW_READ, &par);
          if(ret == 0 && par.status != -1)
            {
              float result = par.result/1024.f; // 10 bit ADC -> 1024
              range = result * 2500/ mV_per_cm / 100;
              rangeMsg.header.stamp = ros::Time::now();
            }
          else
            {
              ROS_WARN("ADC error\n");
              if (par.status == -1)
                ROS_WARN("ADC read error!\n");
            }
        }
      rangeMsg.range = limit(range, min_range, max_range);
      rangePub.publish(rangeMsg);
      r.sleep();
    }

  if(new_kernel)
    fclose(fp);
  else
    close(d);

  return 0;
}

bool check_kernel_version(int kernel_version, int major_version, int minor_version)
{
  struct utsname uts_name;
  if(uname(&uts_name) != 0)
    {
      fprintf(stderr, "Error getting kernel version, assuming new enough kernel!\n");
      return true;
    }
  int a, b, c;
  sscanf(uts_name.release, "%d.%d.%d", &a, &b, &c);
  if(a > kernel_version)
    return true;
  else if(a < kernel_version)
    return false;
  if(b > major_version)
    return true;
  else if(b < major_version)
    return false;
  if(c > minor_version)
    return true;
  else if(c < minor_version)
    return false;
  return true;
}
