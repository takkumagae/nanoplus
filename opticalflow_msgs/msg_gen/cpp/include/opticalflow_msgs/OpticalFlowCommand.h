/* Auto-generated by genmsg_cpp for file /home/root/ros/opticalflow_msgs/msg/OpticalFlowCommand.msg */
#ifndef OPTICALFLOW_MSGS_MESSAGE_OPTICALFLOWCOMMAND_H
#define OPTICALFLOW_MSGS_MESSAGE_OPTICALFLOWCOMMAND_H
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
struct OpticalFlowCommand_ {
  typedef OpticalFlowCommand_<ContainerAllocator> Type;

  OpticalFlowCommand_()
  : vx(0.0)
  , vy(0.0)
  , t(0.0)
  , mode(0)
  {
  }

  OpticalFlowCommand_(const ContainerAllocator& _alloc)
  : vx(0.0)
  , vy(0.0)
  , t(0.0)
  , mode(0)
  {
  }

  typedef double _vx_type;
  double vx;

  typedef double _vy_type;
  double vy;

  typedef double _t_type;
  double t;

  typedef int32_t _mode_type;
  int32_t mode;


private:
  static const char* __s_getDataType_() { return "opticalflow_msgs/OpticalFlowCommand"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "db36d27026ddc82e63c65d723a416964"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float64 vx\n\
float64 vy\n\
float64 t\n\
int32 mode\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, vx);
    ros::serialization::serialize(stream, vy);
    ros::serialization::serialize(stream, t);
    ros::serialization::serialize(stream, mode);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, vx);
    ros::serialization::deserialize(stream, vy);
    ros::serialization::deserialize(stream, t);
    ros::serialization::deserialize(stream, mode);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(vx);
    size += ros::serialization::serializationLength(vy);
    size += ros::serialization::serializationLength(t);
    size += ros::serialization::serializationLength(mode);
    return size;
  }

  typedef boost::shared_ptr< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct OpticalFlowCommand
typedef  ::opticalflow_msgs::OpticalFlowCommand_<std::allocator<void> > OpticalFlowCommand;

typedef boost::shared_ptr< ::opticalflow_msgs::OpticalFlowCommand> OpticalFlowCommandPtr;
typedef boost::shared_ptr< ::opticalflow_msgs::OpticalFlowCommand const> OpticalFlowCommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace opticalflow_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "db36d27026ddc82e63c65d723a416964";
  }

  static const char* value(const  ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xdb36d27026ddc82eULL;
  static const uint64_t static_value2 = 0x63c65d723a416964ULL;
};

template<class ContainerAllocator>
struct DataType< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "opticalflow_msgs/OpticalFlowCommand";
  }

  static const char* value(const  ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 vx\n\
float64 vy\n\
float64 t\n\
int32 mode\n\
\n\
";
  }

  static const char* value(const  ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.vx);
    stream.next(m.vy);
    stream.next(m.t);
    stream.next(m.mode);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct OpticalFlowCommand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::opticalflow_msgs::OpticalFlowCommand_<ContainerAllocator> & v) 
  {
    s << indent << "vx: ";
    Printer<double>::stream(s, indent + "  ", v.vx);
    s << indent << "vy: ";
    Printer<double>::stream(s, indent + "  ", v.vy);
    s << indent << "t: ";
    Printer<double>::stream(s, indent + "  ", v.t);
    s << indent << "mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mode);
  }
};


} // namespace message_operations
} // namespace ros

#endif // OPTICALFLOW_MSGS_MESSAGE_OPTICALFLOWCOMMAND_H

