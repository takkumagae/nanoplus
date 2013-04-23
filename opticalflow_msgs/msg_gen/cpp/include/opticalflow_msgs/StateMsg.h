/* Auto-generated by genmsg_cpp for file /home/root/ros/opticalflow_msgs/msg/StateMsg.msg */
#ifndef OPTICALFLOW_MSGS_MESSAGE_STATEMSG_H
#define OPTICALFLOW_MSGS_MESSAGE_STATEMSG_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace opticalflow_msgs
{
template <class ContainerAllocator>
struct StateMsg_ {
  typedef StateMsg_<ContainerAllocator> Type;

  StateMsg_()
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , vx(0.0)
  , vy(0.0)
  , vicon_vx(0.0)
  , vicon_vy(0.0)
  , sensor_vx(0.0)
  , sensor_vy(0.0)
  , vz(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , des_x(0.0)
  , des_y(0.0)
  , des_z(0.0)
  , des_vx(0.0)
  , des_vy(0.0)
  , des_vz(0.0)
  , des_roll(0.0)
  , des_pitch(0.0)
  , des_yaw(0.0)
  , des_accx(0.0)
  , des_accy(0.0)
  , des_accz(0.0)
  {
  }

  StateMsg_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , vx(0.0)
  , vy(0.0)
  , vicon_vx(0.0)
  , vicon_vy(0.0)
  , sensor_vx(0.0)
  , sensor_vy(0.0)
  , vz(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , des_x(0.0)
  , des_y(0.0)
  , des_z(0.0)
  , des_vx(0.0)
  , des_vy(0.0)
  , des_vz(0.0)
  , des_roll(0.0)
  , des_pitch(0.0)
  , des_yaw(0.0)
  , des_accx(0.0)
  , des_accy(0.0)
  , des_accz(0.0)
  {
  }

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _z_type;
  double z;

  typedef double _vx_type;
  double vx;

  typedef double _vy_type;
  double vy;

  typedef double _vicon_vx_type;
  double vicon_vx;

  typedef double _vicon_vy_type;
  double vicon_vy;

  typedef double _sensor_vx_type;
  double sensor_vx;

  typedef double _sensor_vy_type;
  double sensor_vy;

  typedef double _vz_type;
  double vz;

  typedef double _roll_type;
  double roll;

  typedef double _pitch_type;
  double pitch;

  typedef double _yaw_type;
  double yaw;

  typedef double _des_x_type;
  double des_x;

  typedef double _des_y_type;
  double des_y;

  typedef double _des_z_type;
  double des_z;

  typedef double _des_vx_type;
  double des_vx;

  typedef double _des_vy_type;
  double des_vy;

  typedef double _des_vz_type;
  double des_vz;

  typedef double _des_roll_type;
  double des_roll;

  typedef double _des_pitch_type;
  double des_pitch;

  typedef double _des_yaw_type;
  double des_yaw;

  typedef double _des_accx_type;
  double des_accx;

  typedef double _des_accy_type;
  double des_accy;

  typedef double _des_accz_type;
  double des_accz;


private:
  static const char* __s_getDataType_() { return "opticalflow_msgs/StateMsg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "39ecc4f4366a9d2c85c9ca7bfcf7cd3b"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float64 x\n\
float64 y\n\
float64 z\n\
float64 vx\n\
float64 vy\n\
float64 vicon_vx\n\
float64 vicon_vy\n\
float64 sensor_vx\n\
float64 sensor_vy\n\
float64 vz\n\
float64 roll\n\
float64 pitch\n\
float64 yaw\n\
float64 des_x\n\
float64 des_y\n\
float64 des_z\n\
float64 des_vx\n\
float64 des_vy\n\
float64 des_vz\n\
float64 des_roll\n\
float64 des_pitch\n\
float64 des_yaw\n\
float64 des_accx\n\
float64 des_accy\n\
float64 des_accz\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, x);
    ros::serialization::serialize(stream, y);
    ros::serialization::serialize(stream, z);
    ros::serialization::serialize(stream, vx);
    ros::serialization::serialize(stream, vy);
    ros::serialization::serialize(stream, vicon_vx);
    ros::serialization::serialize(stream, vicon_vy);
    ros::serialization::serialize(stream, sensor_vx);
    ros::serialization::serialize(stream, sensor_vy);
    ros::serialization::serialize(stream, vz);
    ros::serialization::serialize(stream, roll);
    ros::serialization::serialize(stream, pitch);
    ros::serialization::serialize(stream, yaw);
    ros::serialization::serialize(stream, des_x);
    ros::serialization::serialize(stream, des_y);
    ros::serialization::serialize(stream, des_z);
    ros::serialization::serialize(stream, des_vx);
    ros::serialization::serialize(stream, des_vy);
    ros::serialization::serialize(stream, des_vz);
    ros::serialization::serialize(stream, des_roll);
    ros::serialization::serialize(stream, des_pitch);
    ros::serialization::serialize(stream, des_yaw);
    ros::serialization::serialize(stream, des_accx);
    ros::serialization::serialize(stream, des_accy);
    ros::serialization::serialize(stream, des_accz);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    ros::serialization::deserialize(stream, z);
    ros::serialization::deserialize(stream, vx);
    ros::serialization::deserialize(stream, vy);
    ros::serialization::deserialize(stream, vicon_vx);
    ros::serialization::deserialize(stream, vicon_vy);
    ros::serialization::deserialize(stream, sensor_vx);
    ros::serialization::deserialize(stream, sensor_vy);
    ros::serialization::deserialize(stream, vz);
    ros::serialization::deserialize(stream, roll);
    ros::serialization::deserialize(stream, pitch);
    ros::serialization::deserialize(stream, yaw);
    ros::serialization::deserialize(stream, des_x);
    ros::serialization::deserialize(stream, des_y);
    ros::serialization::deserialize(stream, des_z);
    ros::serialization::deserialize(stream, des_vx);
    ros::serialization::deserialize(stream, des_vy);
    ros::serialization::deserialize(stream, des_vz);
    ros::serialization::deserialize(stream, des_roll);
    ros::serialization::deserialize(stream, des_pitch);
    ros::serialization::deserialize(stream, des_yaw);
    ros::serialization::deserialize(stream, des_accx);
    ros::serialization::deserialize(stream, des_accy);
    ros::serialization::deserialize(stream, des_accz);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    size += ros::serialization::serializationLength(z);
    size += ros::serialization::serializationLength(vx);
    size += ros::serialization::serializationLength(vy);
    size += ros::serialization::serializationLength(vicon_vx);
    size += ros::serialization::serializationLength(vicon_vy);
    size += ros::serialization::serializationLength(sensor_vx);
    size += ros::serialization::serializationLength(sensor_vy);
    size += ros::serialization::serializationLength(vz);
    size += ros::serialization::serializationLength(roll);
    size += ros::serialization::serializationLength(pitch);
    size += ros::serialization::serializationLength(yaw);
    size += ros::serialization::serializationLength(des_x);
    size += ros::serialization::serializationLength(des_y);
    size += ros::serialization::serializationLength(des_z);
    size += ros::serialization::serializationLength(des_vx);
    size += ros::serialization::serializationLength(des_vy);
    size += ros::serialization::serializationLength(des_vz);
    size += ros::serialization::serializationLength(des_roll);
    size += ros::serialization::serializationLength(des_pitch);
    size += ros::serialization::serializationLength(des_yaw);
    size += ros::serialization::serializationLength(des_accx);
    size += ros::serialization::serializationLength(des_accy);
    size += ros::serialization::serializationLength(des_accz);
    return size;
  }

  typedef boost::shared_ptr< ::opticalflow_msgs::StateMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opticalflow_msgs::StateMsg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct StateMsg
typedef  ::opticalflow_msgs::StateMsg_<std::allocator<void> > StateMsg;

typedef boost::shared_ptr< ::opticalflow_msgs::StateMsg> StateMsgPtr;
typedef boost::shared_ptr< ::opticalflow_msgs::StateMsg const> StateMsgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::opticalflow_msgs::StateMsg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::opticalflow_msgs::StateMsg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace opticalflow_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::opticalflow_msgs::StateMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::opticalflow_msgs::StateMsg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::opticalflow_msgs::StateMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "39ecc4f4366a9d2c85c9ca7bfcf7cd3b";
  }

  static const char* value(const  ::opticalflow_msgs::StateMsg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x39ecc4f4366a9d2cULL;
  static const uint64_t static_value2 = 0x85c9ca7bfcf7cd3bULL;
};

template<class ContainerAllocator>
struct DataType< ::opticalflow_msgs::StateMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "opticalflow_msgs/StateMsg";
  }

  static const char* value(const  ::opticalflow_msgs::StateMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::opticalflow_msgs::StateMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 x\n\
float64 y\n\
float64 z\n\
float64 vx\n\
float64 vy\n\
float64 vicon_vx\n\
float64 vicon_vy\n\
float64 sensor_vx\n\
float64 sensor_vy\n\
float64 vz\n\
float64 roll\n\
float64 pitch\n\
float64 yaw\n\
float64 des_x\n\
float64 des_y\n\
float64 des_z\n\
float64 des_vx\n\
float64 des_vy\n\
float64 des_vz\n\
float64 des_roll\n\
float64 des_pitch\n\
float64 des_yaw\n\
float64 des_accx\n\
float64 des_accy\n\
float64 des_accz\n\
\n\
";
  }

  static const char* value(const  ::opticalflow_msgs::StateMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::opticalflow_msgs::StateMsg_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::opticalflow_msgs::StateMsg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.vx);
    stream.next(m.vy);
    stream.next(m.vicon_vx);
    stream.next(m.vicon_vy);
    stream.next(m.sensor_vx);
    stream.next(m.sensor_vy);
    stream.next(m.vz);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
    stream.next(m.des_x);
    stream.next(m.des_y);
    stream.next(m.des_z);
    stream.next(m.des_vx);
    stream.next(m.des_vy);
    stream.next(m.des_vz);
    stream.next(m.des_roll);
    stream.next(m.des_pitch);
    stream.next(m.des_yaw);
    stream.next(m.des_accx);
    stream.next(m.des_accy);
    stream.next(m.des_accz);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct StateMsg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opticalflow_msgs::StateMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::opticalflow_msgs::StateMsg_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "vx: ";
    Printer<double>::stream(s, indent + "  ", v.vx);
    s << indent << "vy: ";
    Printer<double>::stream(s, indent + "  ", v.vy);
    s << indent << "vicon_vx: ";
    Printer<double>::stream(s, indent + "  ", v.vicon_vx);
    s << indent << "vicon_vy: ";
    Printer<double>::stream(s, indent + "  ", v.vicon_vy);
    s << indent << "sensor_vx: ";
    Printer<double>::stream(s, indent + "  ", v.sensor_vx);
    s << indent << "sensor_vy: ";
    Printer<double>::stream(s, indent + "  ", v.sensor_vy);
    s << indent << "vz: ";
    Printer<double>::stream(s, indent + "  ", v.vz);
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
    s << indent << "des_x: ";
    Printer<double>::stream(s, indent + "  ", v.des_x);
    s << indent << "des_y: ";
    Printer<double>::stream(s, indent + "  ", v.des_y);
    s << indent << "des_z: ";
    Printer<double>::stream(s, indent + "  ", v.des_z);
    s << indent << "des_vx: ";
    Printer<double>::stream(s, indent + "  ", v.des_vx);
    s << indent << "des_vy: ";
    Printer<double>::stream(s, indent + "  ", v.des_vy);
    s << indent << "des_vz: ";
    Printer<double>::stream(s, indent + "  ", v.des_vz);
    s << indent << "des_roll: ";
    Printer<double>::stream(s, indent + "  ", v.des_roll);
    s << indent << "des_pitch: ";
    Printer<double>::stream(s, indent + "  ", v.des_pitch);
    s << indent << "des_yaw: ";
    Printer<double>::stream(s, indent + "  ", v.des_yaw);
    s << indent << "des_accx: ";
    Printer<double>::stream(s, indent + "  ", v.des_accx);
    s << indent << "des_accy: ";
    Printer<double>::stream(s, indent + "  ", v.des_accy);
    s << indent << "des_accz: ";
    Printer<double>::stream(s, indent + "  ", v.des_accz);
  }
};


} // namespace message_operations
} // namespace ros

#endif // OPTICALFLOW_MSGS_MESSAGE_STATEMSG_H

