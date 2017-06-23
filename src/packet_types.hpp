#ifndef _PACKET_TYPES_HPP_
#define _PACKET_TYPES_HPP_
#include <iodrivers_base/Driver.hpp>

static const int LASER_PACKET_SIZE = 1024;

namespace range_sensor_micro_epsilon {

enum PacketType{
    RawValue = 0,
    ErrorCode, Info
};

const uint32_t REPLY_START = 0x494C4431;
const uint32_t INFO = 0xA04900; //Only 3 bytes
const uint32_t REPLY_END = 0x20200D0A;


}

#endif
