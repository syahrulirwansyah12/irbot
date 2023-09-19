#ifndef _ROS_first_project_GPS_data_h
#define _ROS_first_project_GPS_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace first_project
{

  class GPS_data : public ros::Msg
  {
    public:
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _bearing_type;
      _bearing_type bearing;

    GPS_data():
      longitude(0),
      latitude(0),
      bearing(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_longitude;
      u_longitude.real = this->longitude;
      *(outbuffer + offset + 0) = (u_longitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_longitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_longitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_longitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->longitude);
      union {
        float real;
        uint32_t base;
      } u_latitude;
      u_latitude.real = this->latitude;
      *(outbuffer + offset + 0) = (u_latitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->latitude);
      union {
        float real;
        uint32_t base;
      } u_bearing;
      u_bearing.real = this->bearing;
      *(outbuffer + offset + 0) = (u_bearing.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bearing.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bearing.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bearing.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bearing);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_longitude;
      u_longitude.base = 0;
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->longitude = u_longitude.real;
      offset += sizeof(this->longitude);
      union {
        float real;
        uint32_t base;
      } u_latitude;
      u_latitude.base = 0;
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->latitude = u_latitude.real;
      offset += sizeof(this->latitude);
      union {
        float real;
        uint32_t base;
      } u_bearing;
      u_bearing.base = 0;
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bearing = u_bearing.real;
      offset += sizeof(this->bearing);
     return offset;
    }

    const char * getType(){ return "first_project/GPS_data"; };
    const char * getMD5(){ return "9b30b35dc834a025aeefd8d0b9fbdcb8"; };

  };

}
#endif
