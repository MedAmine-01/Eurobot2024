#ifndef _ROS_SERVICE_RobotCmd_h
#define _ROS_SERVICE_RobotCmd_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace eurobot2024
{

static const char ROBOTCMD[] = "eurobot2024/RobotCmd";

  class RobotCmdRequest : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;
      typedef double _x_type;
      _x_type x;
      typedef double _y_type;
      _y_type y;
      typedef double _phi_type;
      _phi_type phi;
      typedef double _speed_type;
      _speed_type speed;

    RobotCmdRequest():
      command(""),
      x(0),
      y(0),
      phi(0),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
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
      } u_phi;
      u_phi.real = this->phi;
      *(outbuffer + offset + 0) = (u_phi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_phi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_phi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_phi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_phi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_phi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_phi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_phi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->phi);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
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
      } u_phi;
      u_phi.base = 0;
      u_phi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_phi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_phi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_phi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_phi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_phi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_phi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_phi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->phi = u_phi.real;
      offset += sizeof(this->phi);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
     return offset;
    }

    virtual const char * getType() override { return ROBOTCMD; };
    virtual const char * getMD5() override { return "95d8acd102ee5c8922ed9b62c74b419d"; };

  };

  class RobotCmdResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    RobotCmdResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    virtual const char * getType() override { return ROBOTCMD; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class RobotCmd {
    public:
    typedef RobotCmdRequest Request;
    typedef RobotCmdResponse Response;
  };

}
#endif
