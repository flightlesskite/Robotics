// Generated by gencpp from file myturtlebot/TurtlebotStatus.msg
// DO NOT EDIT!


#ifndef MYTURTLEBOT_MESSAGE_TURTLEBOTSTATUS_H
#define MYTURTLEBOT_MESSAGE_TURTLEBOTSTATUS_H

#include <ros/service_traits.h>


#include <myturtlebot/TurtlebotStatusRequest.h>
#include <myturtlebot/TurtlebotStatusResponse.h>


namespace myturtlebot
{

struct TurtlebotStatus
{

typedef TurtlebotStatusRequest Request;
typedef TurtlebotStatusResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct TurtlebotStatus
} // namespace myturtlebot


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::myturtlebot::TurtlebotStatus > {
  static const char* value()
  {
    return "5402a31cf3717369ede1e7580132916f";
  }

  static const char* value(const ::myturtlebot::TurtlebotStatus&) { return value(); }
};

template<>
struct DataType< ::myturtlebot::TurtlebotStatus > {
  static const char* value()
  {
    return "myturtlebot/TurtlebotStatus";
  }

  static const char* value(const ::myturtlebot::TurtlebotStatus&) { return value(); }
};


// service_traits::MD5Sum< ::myturtlebot::TurtlebotStatusRequest> should match 
// service_traits::MD5Sum< ::myturtlebot::TurtlebotStatus > 
template<>
struct MD5Sum< ::myturtlebot::TurtlebotStatusRequest>
{
  static const char* value()
  {
    return MD5Sum< ::myturtlebot::TurtlebotStatus >::value();
  }
  static const char* value(const ::myturtlebot::TurtlebotStatusRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::myturtlebot::TurtlebotStatusRequest> should match 
// service_traits::DataType< ::myturtlebot::TurtlebotStatus > 
template<>
struct DataType< ::myturtlebot::TurtlebotStatusRequest>
{
  static const char* value()
  {
    return DataType< ::myturtlebot::TurtlebotStatus >::value();
  }
  static const char* value(const ::myturtlebot::TurtlebotStatusRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::myturtlebot::TurtlebotStatusResponse> should match 
// service_traits::MD5Sum< ::myturtlebot::TurtlebotStatus > 
template<>
struct MD5Sum< ::myturtlebot::TurtlebotStatusResponse>
{
  static const char* value()
  {
    return MD5Sum< ::myturtlebot::TurtlebotStatus >::value();
  }
  static const char* value(const ::myturtlebot::TurtlebotStatusResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::myturtlebot::TurtlebotStatusResponse> should match 
// service_traits::DataType< ::myturtlebot::TurtlebotStatus > 
template<>
struct DataType< ::myturtlebot::TurtlebotStatusResponse>
{
  static const char* value()
  {
    return DataType< ::myturtlebot::TurtlebotStatus >::value();
  }
  static const char* value(const ::myturtlebot::TurtlebotStatusResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MYTURTLEBOT_MESSAGE_TURTLEBOTSTATUS_H
