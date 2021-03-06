/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-multimaster-experimental-0.2.1/debian/ros-groovy-multimaster-experimental/opt/ros/groovy/stacks/multimaster_experimental/app_manager/msg/App.msg */
#ifndef APP_MANAGER_MESSAGE_APP_H
#define APP_MANAGER_MESSAGE_APP_H
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

#include "app_manager/Icon.h"
#include "app_manager/ClientApp.h"

namespace app_manager
{
template <class ContainerAllocator>
struct App_ {
  typedef App_<ContainerAllocator> Type;

  App_()
  : name()
  , display_name()
  , icon()
  , client_apps()
  {
  }

  App_(const ContainerAllocator& _alloc)
  : name(_alloc)
  , display_name(_alloc)
  , icon(_alloc)
  , client_apps(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _display_name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  display_name;

  typedef  ::app_manager::Icon_<ContainerAllocator>  _icon_type;
   ::app_manager::Icon_<ContainerAllocator>  icon;

  typedef std::vector< ::app_manager::ClientApp_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::app_manager::ClientApp_<ContainerAllocator> >::other >  _client_apps_type;
  std::vector< ::app_manager::ClientApp_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::app_manager::ClientApp_<ContainerAllocator> >::other >  client_apps;


  typedef boost::shared_ptr< ::app_manager::App_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::app_manager::App_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct App
typedef  ::app_manager::App_<std::allocator<void> > App;

typedef boost::shared_ptr< ::app_manager::App> AppPtr;
typedef boost::shared_ptr< ::app_manager::App const> AppConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::app_manager::App_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::app_manager::App_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace app_manager

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::app_manager::App_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::app_manager::App_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::app_manager::App_<ContainerAllocator> > {
  static const char* value() 
  {
    return "643c1db5f71b615a47789ff5e190811e";
  }

  static const char* value(const  ::app_manager::App_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x643c1db5f71b615aULL;
  static const uint64_t static_value2 = 0x47789ff5e190811eULL;
};

template<class ContainerAllocator>
struct DataType< ::app_manager::App_<ContainerAllocator> > {
  static const char* value() 
  {
    return "app_manager/App";
  }

  static const char* value(const  ::app_manager::App_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::app_manager::App_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# app name\n\
string name\n\
# user-friendly display name of application\n\
string display_name\n\
# icon for showing app\n\
Icon icon\n\
# ordered list (by preference) of client applications to interact with this robot app.  \n\
ClientApp[] client_apps\n\
\n\
================================================================================\n\
MSG: app_manager/Icon\n\
# Image data format.  \"jpeg\" or \"png\"\n\
string format\n\
\n\
# Image data.\n\
uint8[] data\n\
\n\
================================================================================\n\
MSG: app_manager/ClientApp\n\
# like \"android\" or \"web\" or \"linux\"\n\
string client_type\n\
\n\
# like \"intent = ros.android.teleop\" and \"accelerometer = true\", used to choose which ClientApp to use\n\
KeyValue[] manager_data\n\
\n\
# parameters which just get passed through to the client app.\n\
KeyValue[] app_data\n\
\n\
================================================================================\n\
MSG: app_manager/KeyValue\n\
string key\n\
string value\n\
\n\
";
  }

  static const char* value(const  ::app_manager::App_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::app_manager::App_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.display_name);
    stream.next(m.icon);
    stream.next(m.client_apps);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct App_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::app_manager::App_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::app_manager::App_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "display_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.display_name);
    s << indent << "icon: ";
s << std::endl;
    Printer< ::app_manager::Icon_<ContainerAllocator> >::stream(s, indent + "  ", v.icon);
    s << indent << "client_apps[]" << std::endl;
    for (size_t i = 0; i < v.client_apps.size(); ++i)
    {
      s << indent << "  client_apps[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::app_manager::ClientApp_<ContainerAllocator> >::stream(s, indent + "    ", v.client_apps[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // APP_MANAGER_MESSAGE_APP_H

