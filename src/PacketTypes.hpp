#ifndef _PACKET_TYPES_HPP_
#define _PACKET_TYPES_HPP_
#include <iodrivers_base/Driver.hpp>


namespace range_sensor_micro_epsilon {

static const int LASER_PACKET_SIZE = 1024;

struct ErrorStats{
    uint64_t no_object_detected;
    uint64_t too_close_to_the_sensor;
    uint64_t too_far_from_the_sensor;
    uint64_t target_can_not_be_evaluated;
    uint64_t target_moves_towards_the_sensor;
    uint64_t target_moves_away_from_sensor;
    uint64_t command_packets;
    uint64_t unknown;
    void clear(){
        no_object_detected = 0;
        too_close_to_the_sensor = 0;
        too_far_from_the_sensor = 0;
        target_can_not_be_evaluated = 0;
        target_moves_towards_the_sensor = 0;
        target_moves_away_from_sensor = 0;
        command_packets = 0;
        unknown = 0;
    }
};

const uint32_t REPLY_START = 0x494C4431;
const uint32_t INFO = 0xA04900; //Only 3 bytes
const uint32_t REPLY_END = 0x20200D0A;


}

#endif
