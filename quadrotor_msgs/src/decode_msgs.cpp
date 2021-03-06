#include "quadrotor_msgs/decode_msgs.h"
#include <quadrotor_msgs/OutputData.h>
#include <quadrotor_msgs/StatusData.h>
#include <quadrotor_msgs/comm_types.h>
#include <Eigen/Geometry>

namespace quadrotor_msgs
{

bool decodeOutputData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::OutputData &output)
{
  struct OUTPUT_DATA output_data;
  if(data.size() != sizeof(output_data))
    return false;

  memcpy(&output_data, &data[0], sizeof(output_data));
  output.loop_rate = output_data.loop_rate;
  output.voltage = output_data.voltage/1e3;

  const double roll = output_data.roll/1e2 * M_PI/180;
  const double pitch = output_data.pitch/1e2 * M_PI/180;
  const double yaw = output_data.yaw/1e2 * M_PI/180;
  // Asctec (2012 firmware) uses  Z-Y-X convention
  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  output.orientation.w = q.w();
  output.orientation.x = q.x();
  output.orientation.y = q.y();
  output.orientation.z = q.z();

  output.angular_velocity.x = output_data.ang_vel[0]*0.0154*M_PI/180;
  output.angular_velocity.y = output_data.ang_vel[1]*0.0154*M_PI/180;
  output.angular_velocity.z = output_data.ang_vel[2]*0.0154*M_PI/180;

  output.linear_acceleration.x = output_data.acc[0]/1e3 * 9.81;
  output.linear_acceleration.y = output_data.acc[1]/1e3 * 9.81;
  output.linear_acceleration.z = output_data.acc[2]/1e3 * 9.81;

  output.pressure_dheight = output_data.dheight/1e3;
  output.pressure_height = output_data.height/1e3;

  output.magnetic_field.x = output_data.mag[0]/2500.0;
  output.magnetic_field.y = output_data.mag[1]/2500.0;
  output.magnetic_field.z = output_data.mag[2]/2500.0;

  for(int i = 0; i < 8; i++)
  {
    output.radio_channel[i] = output_data.radio[i];
  }
  //for(int i = 0; i < 4; i++)
  //  output.motor_rpm[i] = output_data.rpm[i];

  output.seq = output_data.seq;

  return true;
}

bool decodeStatusData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::StatusData &status)
{
  struct STATUS_DATA status_data;
  if(data.size() != sizeof(status_data))
    return false;
  memcpy(&status_data, &data[0], sizeof(status_data));

  status.loop_rate = status_data.loop_rate;
  status.voltage = status_data.voltage/1e3;
  status.seq = status_data.seq;

  return true;
}

}
