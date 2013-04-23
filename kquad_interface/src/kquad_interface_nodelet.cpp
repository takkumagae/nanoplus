#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <kQuadInterface.hh>
#include <quadrotor_msgs/TRPYCommand.h>
#include <quadrotor_msgs/OutputData.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <list>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Vector3Stamped.h>

class kQuadInterfaceNodelet : public nodelet::Nodelet
{
 public:
  ~kQuadInterfaceNodelet(void);
  void onInit(void);

 private:
  void trpy_command_cb(const quadrotor_msgs::TRPYCommand::ConstPtr &msg);
  void output_thread(void);

  uint8_t quad_id_, quad_type_, channel_;
  kQuadInterface kqi_;

  ros::Subscriber trpy_command_sub_;
  ros::Publisher output_pub_;
  ros::Publisher status_pub_;

  volatile bool output_thread_running_;
  boost::shared_ptr<boost::thread> output_thread_ptr_;
};

void kQuadInterfaceNodelet::onInit(void)
{
  ros::NodeHandle n(getMTPrivateNodeHandle());

  std::string device;
  int baud_rate;
  n.param("serial_device", device, std::string("/dev/ttyUSB0"));
  n.param("serial_baud_rate", baud_rate, 921600);

  int quad_id, quad_type, channel;
  n.param("quad_id", quad_id, 11);
  n.param("quad_type", quad_type, 0);
  n.param("channel", channel, 1);
  quad_id_ = quad_id;
  quad_type_ = quad_type;
  channel_ = channel;

  trpy_command_sub_ = n.subscribe("trpy_command", 10, &kQuadInterfaceNodelet::trpy_command_cb, this,
                                  ros::TransportHints().udp().tcpNoDelay());

  output_pub_ = n.advertise<quadrotor_msgs::OutputData>("output", 10);
  status_pub_ = n.advertise<geometry_msgs::Vector3Stamped>("status", 10);


  if(kqi_.Connect(device.c_str(), baud_rate))
  {
    NODELET_FATAL("could not connect to the device");
  }

  if (kqi_.StartSendThread())
  {
    NODELET_FATAL("could not start send thread");
  }

  if (kqi_.StartRecvThread())
  {
    NODELET_FATAL("could not start receive thread");
  }

  // spawn data output thread
  output_thread_running_ = true;
  output_thread_ptr_ = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&kQuadInterfaceNodelet::output_thread, this)));
}

void kQuadInterfaceNodelet::trpy_command_cb(const quadrotor_msgs::TRPYCommand::ConstPtr &msg)
{
  kqi_.SendQuadCmd1(quad_id_, quad_type_, channel_, msg->thrust*1000/9.81f, msg->roll, msg->pitch, msg->yaw);
}

void kQuadInterfaceNodelet::output_thread(void)
{
  uint8_t rc_channels[8] = {127,127,127,127,127,127,127,127};
  while(output_thread_running_)
  {
    std::list<ImuFiltData> if_data;
    std::list<RcData> rc_data_list;
    int nmsg = kqi_.GetImuFiltData(if_data);
    int nmsg_rc = kqi_.GetRcData(rc_data_list);
    if(nmsg_rc > 0)
    {
      RcData &rc_data = rc_data_list.front();
      for(unsigned int i = 0; i < 8; i++)
      {
        rc_channels[i] = rc_data.data[i]/4;
      }
    }
    if(nmsg > 0)
    {
      ImuFiltData &imu_data = if_data.front();
      quadrotor_msgs::OutputData::Ptr output_msg(new quadrotor_msgs::OutputData);
      output_msg->header.stamp = ros::Time(imu_data.tpc);

      Eigen::Quaternionf q = Eigen::AngleAxisf(imu_data.yaw, Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(imu_data.pitch, Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(imu_data.roll, Eigen::Vector3f::UnitX());
      output_msg->orientation.x = q.x();
      output_msg->orientation.y = q.y();
      output_msg->orientation.z = q.z();
      output_msg->orientation.w = q.w();

      output_msg->angular_velocity.x = imu_data.wroll;
      output_msg->angular_velocity.y = imu_data.wpitch;
      output_msg->angular_velocity.z = imu_data.wyaw;

      output_msg->linear_acceleration.x = imu_data.ax * 9.81;
      output_msg->linear_acceleration.y = imu_data.ay * 9.81;
      output_msg->linear_acceleration.z = imu_data.az * 9.81;

      for(unsigned int i = 0; i < 8; i++)
        output_msg->radio_channel[i] = rc_channels[i];

      output_pub_.publish(output_msg);
    }

    std::list<QuadStatusData> status_list;
    int nmsg_status = kqi_.GetQuadStatusData(status_list);
    if(nmsg_status > 0)
    {
      QuadStatusData &status_data = status_list.front();
      geometry_msgs::Vector3Stamped::Ptr status_msg(new geometry_msgs::Vector3Stamped);
      status_msg->header.stamp = ros::Time(status_data.tpc);
      status_msg->vector.x = status_data.voltage;
      status_msg->vector.y = status_data.current;
      status_pub_.publish(status_msg);
    }

    //boost::this_thread::sleep(boost::posix_time::milliseconds(2));
    usleep(2000);
  }
}

kQuadInterfaceNodelet::~kQuadInterfaceNodelet(void)
{
  if(output_thread_running_)
  {
    output_thread_running_ = false;
    output_thread_ptr_->join();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(kquad_interface, kQuadInterfaceNodelet, kQuadInterfaceNodelet, nodelet::Nodelet);
