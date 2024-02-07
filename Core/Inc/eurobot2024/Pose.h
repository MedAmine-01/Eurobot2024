#ifndef _ROS_eurobot2024_Pose_h
#define _ROS_eurobot2024_Pose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace eurobot2024
{

  class Pose : public ros::Msg
  {
    public:
      typedef double _x_type;
      _x_type x;
      typedef double _y_type;
      _y_type y;
      typedef double _phi_deg_type;
      _phi_deg_type phi_deg;

    Pose():
      x(0),
      y(0),
      phi_deg(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_phi_deg;
      u_phi_deg.real = this->phi_deg;
      *(outbuffer + offset + 0) = (u_phi_deg.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_phi_deg.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_phi_deg.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_phi_deg.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_phi_deg.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_phi_deg.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_phi_deg.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_phi_deg.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->phi_deg);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_phi_deg;
      u_phi_deg.base = 0;
      u_phi_deg.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_phi_deg.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_phi_deg.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_phi_deg.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_phi_deg.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_phi_deg.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_phi_deg.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_phi_deg.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->phi_deg = u_phi_deg.real;
      offset += sizeof(this->phi_deg);
     return offset;
    }

    virtual const char * getType() override { return "eurobot2024/Pose"; };
    virtual const char * getMD5() override { return "2845a7293ff5c13188732b629d0955bc"; };

  };

}
#endif
