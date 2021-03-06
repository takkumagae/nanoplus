/* Auto-generated by genmsg_cpp for file /home/root/ros/opticalflow_msgs/msg/Goal.msg */
#ifndef OPTICALFLOW_MSGS_MESSAGE_GOAL_H
#define OPTICALFLOW_MSGS_MESSAGE_GOAL_H
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
struct Goal_ {
  typedef Goal_<ContainerAllocator> Type;

  Goal_()
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , v(0.0)
  , mode(0.0)
  , use_sensor(false)
  {
  }

  Goal_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , v(0.0)
  , mode(0.0)
  , use_sensor(false)
  {
  }

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _z_type;
  double z;

  typedef double _v_type;
  double v;

  typedef double _mode_type;
  double mode;

  typedef uint8_t _use_sensor_type;
  uint8_t use_sensor;


private:
  static const char* __s_getDataType_() { return "opticalflow_msgs/Goal"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "eff9ed298a9453be33459118f380c353"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float64 x\n\
float64 y\n\
float64 z\n\
float64 v\n\
float64 mode\n\
bool use_sensor\n\
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
    ros::serialization::serialize(stream, v);
    ros::serialization::serialize(stream, mode);
    ros::serialization::serialize(stream, use_sensor);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    ros::serialization::deserialize(stream, z);
    ros::serialization::deserialize(stream, v);
    ros::serialization::deserialize(stream, mode);
    ros::serialization::deserialize(stream, use_sensor);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    size += ros::serialization::serializationLength(z);
    size += ros::serialization::serializationLength(v);
    size += ros::serialization::serializationLength(mode);
    size += ros::serialization::serializationLength(use_sensor);
    return size;
  }

  typedef boost::shared_ptr< ::opticalflow_msgs::Goal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opticalflow_msgs::Goal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Goal
typedef  ::opticalflow_msgs::Goal_<std::allocator<void> > Goal;

typedef boost::shared_ptr< ::opticalflow_msgs::Goal> GoalPtr;
typedef boost::shared_ptr< ::opticalflow_msgs::Goal const> GoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::opticalflow_msgs::Goal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::opticalflow_msgs::Goal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace opticalflow_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::opticalflow_msgs::Goal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::opticalflow_msgs::Goal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::opticalflow_msgs::Goal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "eff9ed298a9453be33459118f380c353";
  }

  static const char* value(const  ::opticalflow_msgs::Goal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xeff9ed298a9453beULL;
  static const uint64_t static_value2 = 0x33459118f380c353ULL;
};

template<class ContainerAllocator>
struct DataType< ::opticalflow_msgs::Goal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "opticalflow_msgs/Goal";
  }

  static const char* value(const  ::opticalflow_msgs::Goal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::opticalflow_msgs::Goal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 x\n\
float64 y\n\
float64 z\n\
float64 v\n\
float64 mode\n\
bool use_sensor\n\
\n\
";
  }

  static const char* value(const  ::opticalflow_msgs::Goal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::opticalflow_msgs::Goal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::opticalflow_msgs::Goal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.v);
    stream.next(m.mode);
    stream.next(m.use_sensor);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Goal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opticalflow_msgs::Goal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::opticalflow_msgs::Goal_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "mode: ";
    Printer<double>::stream(s, indent + "  ", v.mode);
    s << indent << "use_sensor: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_sensor);
  }
};


} // namespace message_operations
} // namespace ros

#endif // OPTICALFLOW_MSGS_MESSAGE_GOAL_H

