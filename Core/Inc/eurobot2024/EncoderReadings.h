#ifndef _ROS_eurobot2024_EncoderReadings_h
#define _ROS_eurobot2024_EncoderReadings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace eurobot2024
{

  class EncoderReadings : public ros::Msg
  {
    public:
      typedef double _leftCount_type;
      _leftCount_type leftCount;
      typedef double _rightCount_type;
      _rightCount_type rightCount;

    EncoderReadings():
      leftCount(0),
      rightCount(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_leftCount;
      u_leftCount.real = this->leftCount;
      *(outbuffer + offset + 0) = (u_leftCount.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftCount.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftCount.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftCount.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_leftCount.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_leftCount.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_leftCount.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_leftCount.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->leftCount);
      union {
        double real;
        uint64_t base;
      } u_rightCount;
      u_rightCount.real = this->rightCount;
      *(outbuffer + offset + 0) = (u_rightCount.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightCount.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightCount.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightCount.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rightCount.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rightCount.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rightCount.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rightCount.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rightCount);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_leftCount;
      u_leftCount.base = 0;
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_leftCount.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->leftCount = u_leftCount.real;
      offset += sizeof(this->leftCount);
      union {
        double real;
        uint64_t base;
      } u_rightCount;
      u_rightCount.base = 0;
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rightCount.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rightCount = u_rightCount.real;
      offset += sizeof(this->rightCount);
     return offset;
    }

    virtual const char * getType() override { return "eurobot2024/EncoderReadings"; };
    virtual const char * getMD5() override { return "0a660674af4e5b9453017daeb3a5836e"; };

  };

}
#endif
