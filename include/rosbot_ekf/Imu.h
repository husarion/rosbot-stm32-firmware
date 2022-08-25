// #ifndef _ROS_rosbot_ekf_Imu_h
// #define _ROS_rosbot_ekf_Imu_h

// #include <stdint.h>
// #include <string.h>
// #include <stdlib.h>
// #include "ros/msg.h"
// #include "std_msgs/Header.h"
// #include "geometry_msgs/Quaternion.h"

// namespace rosbot_ekf
// {

//   class Imu : public ros::Msg
//   {
//     public:
//       typedef std_msgs::Header _header_type;
//       _header_type header;
//       typedef geometry_msgs::Quaternion _orientation_type;
//       _orientation_type orientation;
//       float angular_velocity[3];
//       float linear_acceleration[3];

//     Imu():
//       header(),
//       orientation(),
//       angular_velocity(),
//       linear_acceleration()
//     {
//     }

//     virtual int serialize(unsigned char *outbuffer) const
//     {
//       int offset = 0;
//       offset += this->header.serialize(outbuffer + offset);
//       offset += this->orientation.serialize(outbuffer + offset);
//       for( uint32_t i = 0; i < 3; i++){
//       union {
//         float real;
//         uint32_t base;
//       } u_angular_velocityi;
//       u_angular_velocityi.real = this->angular_velocity[i];
//       *(outbuffer + offset + 0) = (u_angular_velocityi.base >> (8 * 0)) & 0xFF;
//       *(outbuffer + offset + 1) = (u_angular_velocityi.base >> (8 * 1)) & 0xFF;
//       *(outbuffer + offset + 2) = (u_angular_velocityi.base >> (8 * 2)) & 0xFF;
//       *(outbuffer + offset + 3) = (u_angular_velocityi.base >> (8 * 3)) & 0xFF;
//       offset += sizeof(this->angular_velocity[i]);
//       }
//       for( uint32_t i = 0; i < 3; i++){
//       union {
//         float real;
//         uint32_t base;
//       } u_linear_accelerationi;
//       u_linear_accelerationi.real = this->linear_acceleration[i];
//       *(outbuffer + offset + 0) = (u_linear_accelerationi.base >> (8 * 0)) & 0xFF;
//       *(outbuffer + offset + 1) = (u_linear_accelerationi.base >> (8 * 1)) & 0xFF;
//       *(outbuffer + offset + 2) = (u_linear_accelerationi.base >> (8 * 2)) & 0xFF;
//       *(outbuffer + offset + 3) = (u_linear_accelerationi.base >> (8 * 3)) & 0xFF;
//       offset += sizeof(this->linear_acceleration[i]);
//       }
//       return offset;
//     }

//     virtual int deserialize(unsigned char *inbuffer)
//     {
//       int offset = 0;
//       offset += this->header.deserialize(inbuffer + offset);
//       offset += this->orientation.deserialize(inbuffer + offset);
//       for( uint32_t i = 0; i < 3; i++){
//       union {
//         float real;
//         uint32_t base;
//       } u_angular_velocityi;
//       u_angular_velocityi.base = 0;
//       u_angular_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
//       u_angular_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
//       u_angular_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
//       u_angular_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
//       this->angular_velocity[i] = u_angular_velocityi.real;
//       offset += sizeof(this->angular_velocity[i]);
//       }
//       for( uint32_t i = 0; i < 3; i++){
//       union {
//         float real;
//         uint32_t base;
//       } u_linear_accelerationi;
//       u_linear_accelerationi.base = 0;
//       u_linear_accelerationi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
//       u_linear_accelerationi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
//       u_linear_accelerationi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
//       u_linear_accelerationi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
//       this->linear_acceleration[i] = u_linear_accelerationi.real;
//       offset += sizeof(this->linear_acceleration[i]);
//       }
//      return offset;
//     }

//     const char * getType(){ return "rosbot_ekf/Imu"; };
//     const char * getMD5(){ return "3d83bdcabfe2927ed38c36f102a9f646"; };

//   };

// }
// #endif
