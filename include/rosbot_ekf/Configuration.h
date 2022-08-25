#pragma once
// #ifndef _ROS_SERVICE_Configuration_h
// #define _ROS_SERVICE_Configuration_h
// #include <stdint.h>
// #include <string.h>
// #include <stdlib.h>
// // #include "ros/msg.h"

namespace rosbot_ekf
{

// static const char CONFIGURATION[] = "rosbot_ekf/Configuration";

//   class ConfigurationRequest : public ros::Msg
//   {
//     public:
//       typedef const char* _command_type;
//       _command_type command;
//       typedef const char* _data_type;
//       _data_type data;

//     ConfigurationRequest():
//       command(""),
//       data("")
//     {
//     }

//     virtual int serialize(unsigned char *outbuffer) const
//     {
//       int offset = 0;
//       uint32_t length_command = strlen(this->command);
//       varToArr(outbuffer + offset, length_command);
//       offset += 4;
//       memcpy(outbuffer + offset, this->command, length_command);
//       offset += length_command;
//       uint32_t length_data = strlen(this->data);
//       varToArr(outbuffer + offset, length_data);
//       offset += 4;
//       memcpy(outbuffer + offset, this->data, length_data);
//       offset += length_data;
//       return offset;
//     }

//     virtual int deserialize(unsigned char *inbuffer)
//     {
//       int offset = 0;
//       uint32_t length_command;
//       arrToVar(length_command, (inbuffer + offset));
//       offset += 4;
//       for(unsigned int k= offset; k< offset+length_command; ++k){
//           inbuffer[k-1]=inbuffer[k];
//       }
//       inbuffer[offset+length_command-1]=0;
//       this->command = (char *)(inbuffer + offset-1);
//       offset += length_command;
//       uint32_t length_data;
//       arrToVar(length_data, (inbuffer + offset));
//       offset += 4;
//       for(unsigned int k= offset; k< offset+length_data; ++k){
//           inbuffer[k-1]=inbuffer[k];
//       }
//       inbuffer[offset+length_data-1]=0;
//       this->data = (char *)(inbuffer + offset-1);
//       offset += length_data;
//      return offset;
//     }

//     const char * getType(){ return CONFIGURATION; };
//     const char * getMD5(){ return "b116973260063b7b02b501a288810d3d"; };

//   };

// class ConfigurationResponse : public ros::Msg
//   {
//     public:
//       typedef const char* _data_type;
//       _data_type data;
//       typedef uint8_t _result_type;
//       _result_type result;
//       enum { SUCCESS = 0 };
//       enum { FAILURE = 1 };
//       enum { COMMAND_NOT_FOUND = 2 };

//     ConfigurationResponse():
//       data(""),
//       result(0)
//     {
//     }

//     virtual int serialize(unsigned char *outbuffer) const
//     {
//       int offset = 0;
//       uint32_t length_data = strlen(this->data);
//       varToArr(outbuffer + offset, length_data);
//       offset += 4;
//       memcpy(outbuffer + offset, this->data, length_data);
//       offset += length_data;
//       *(outbuffer + offset + 0) = (this->result >> (8 * 0)) & 0xFF;
//       offset += sizeof(this->result);
//       return offset;
//     }

//     virtual int deserialize(unsigned char *inbuffer)
//     {
//       int offset = 0;
//       uint32_t length_data;
//       arrToVar(length_data, (inbuffer + offset));
//       offset += 4;
//       for(unsigned int k= offset; k< offset+length_data; ++k){
//           inbuffer[k-1]=inbuffer[k];
//       }
//       inbuffer[offset+length_data-1]=0;
//       this->data = (char *)(inbuffer + offset-1);
//       offset += length_data;
//       this->result =  ((uint8_t) (*(inbuffer + offset)));
//       offset += sizeof(this->result);
//      return offset;
//     }

//     const char * getType(){ return CONFIGURATION; };
//     const char * getMD5(){ return "d26ad64e1f52355ee24245b602a171ba"; };

//   };


class ConfigurationRequest {
};

class ConfigurationResponse {
   public:
    enum { SUCCESS = 0 };
    enum { FAILURE = 1 };
    enum { COMMAND_NOT_FOUND = 2 };
};

class Configuration {
   public:
    typedef ConfigurationRequest Request;
    typedef ConfigurationResponse Response;
};
}
// #endif
