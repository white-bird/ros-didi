/* Auto-generated by genmsg_cpp for file /opt/ros/kinetic/share/workspace/yingzz/msg/pos.msg */
#ifndef YINGZZ_MESSAGE_POS_H
#define YINGZZ_MESSAGE_POS_H
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

#include "std_msgs/Header.h"

namespace yingzz
{
template <class ContainerAllocator>
struct pos_ {
  typedef pos_<ContainerAllocator> Type;

  pos_()
  : header()
  , timestamp(0)
  , id(0)
  , type(0)
  , x(0)
  , y(0)
  , z(0)
  , l(0)
  , w(0)
  , h(0)
  , roll(0)
  , pitch(0)
  , yaw(0)
  {
  }

  pos_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , timestamp(0)
  , id(0)
  , type(0)
  , x(0)
  , y(0)
  , z(0)
  , l(0)
  , w(0)
  , h(0)
  , roll(0)
  , pitch(0)
  , yaw(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint64_t _timestamp_type;
  uint64_t timestamp;

  typedef uint32_t _id_type;
  uint32_t id;

  typedef uint32_t _type_type;
  uint32_t type;

  typedef uint32_t _x_type;
  uint32_t x;

  typedef uint32_t _y_type;
  uint32_t y;

  typedef uint32_t _z_type;
  uint32_t z;

  typedef uint32_t _l_type;
  uint32_t l;

  typedef uint32_t _w_type;
  uint32_t w;

  typedef uint32_t _h_type;
  uint32_t h;

  typedef uint32_t _roll_type;
  uint32_t roll;

  typedef uint32_t _pitch_type;
  uint32_t pitch;

  typedef uint32_t _yaw_type;
  uint32_t yaw;


  typedef boost::shared_ptr< ::yingzz::pos_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::yingzz::pos_<ContainerAllocator>  const> ConstPtr;
}; // struct pos
typedef  ::yingzz::pos_<std::allocator<void> > pos;

typedef boost::shared_ptr< ::yingzz::pos> posPtr;
typedef boost::shared_ptr< ::yingzz::pos const> posConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::yingzz::pos_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::yingzz::pos_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace yingzz

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::yingzz::pos_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::yingzz::pos_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::yingzz::pos_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9e29d3ec8c20e406ea48a821affcb6ea";
  }

  static const char* value(const  ::yingzz::pos_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9e29d3ec8c20e406ULL;
  static const uint64_t static_value2 = 0xea48a821affcb6eaULL;
};

template<class ContainerAllocator>
struct DataType< ::yingzz::pos_<ContainerAllocator> > {
  static const char* value() 
  {
    return "yingzz/pos";
  }

  static const char* value(const  ::yingzz::pos_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::yingzz::pos_<ContainerAllocator> > {
  static const char* value() 
  {
    return "std_msgs/Header header\n\
uint64 timestamp\n\
uint32 id\n\
uint32 type\n\
uint32 x\n\
uint32 y\n\
uint32 z\n\
uint32 l\n\
uint32 w\n\
uint32 h\n\
uint32 roll\n\
uint32 pitch\n\
uint32 yaw\n\
\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::yingzz::pos_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::yingzz::pos_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::yingzz::pos_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::yingzz::pos_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.timestamp);
    stream.next(m.id);
    stream.next(m.type);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.l);
    stream.next(m.w);
    stream.next(m.h);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct pos_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::yingzz::pos_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::yingzz::pos_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "timestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.type);
    s << indent << "x: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.z);
    s << indent << "l: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.l);
    s << indent << "w: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.w);
    s << indent << "h: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.h);
    s << indent << "roll: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.yaw);
  }
};


} // namespace message_operations
} // namespace ros

#endif // YINGZZ_MESSAGE_POS_H

