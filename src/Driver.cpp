#include "Driver.hpp"
#include "packet_types.hpp"
#include <iostream>
#include <cstring>
#include <base-logging/Logging.hpp>

using namespace std;
using namespace range_sensor_micro_epsilon;

inline uint32_t getWord(const uint8_t* buffer){
    return be32toh(*(reinterpret_cast<const uint32_t*> (buffer)));
}

RangeSensor::RangeSensor():
    iodrivers_base::Driver(1024),range_value(0),smr(0.2),mr(0.6)
{
}

RangeSensor::~RangeSensor()
{
}

uint16_t RangeSensor::rawToDVO(const uint8_t* buffer){
    return (buffer[0] & 0b01111111)*(1<<7) + buffer[1];
}

double RangeSensor::DVOToMeasurement(uint16_t dvo){
    return  ((static_cast<double>(dvo))*1.02/16368 - 0.01)*mr + smr;
}

double RangeSensor::rawToMeasurement(const uint8_t* buffer){
    return  DVOToMeasurement(rawToDVO(buffer));
}

bool RangeSensor::readPacket(int timeout)
{
    size_t size = Driver::readPacket(
            msg,
            LASER_PACKET_SIZE,
            base::Time::fromMilliseconds(timeout));
    if(get_word(msg) == REPLY_START)
        return false;

    if( (size%2) != 0)
        throw std::runtime_error("extractPacket has extracted an invalid package!");  //this should never happen

    LOG_DEBUG_S << "reading packet with "<< size/2 << " measurements";

    range_value.clear();
    range_value.reserve(size/2);

    for(int i = 0; i < size-1; i+=2)
        if((~msg[i] | msg[i+1]) & 0b10000000)
            throw std::runtime_error("extractPacket has extracted an invalid package!");
        else{
            uint16_t dvo = rawToDVO(&msg[i]);
            if(dvo > 16367)
                continue;
            range_value.push_back(DVOToMeasurement(dvo));
        }

    return !range_value.empty();
}

int RangeSensor::extractPacket(const uint8_t *buffer, size_t buffer_size) const
{
    if(buffer_size < 2){ //throw std::runtime_error("Packet too small to contain range values.");
        //std::cout << "Packet too small to contain range values. "<< std::endl;
        return 0;
    }

    int start = findFirstWord(buffer,buffer_size,&REPLY_START);
    int end = findFirstWord(buffer,buffer_size,&REPLY_END);

    if (end < start)
    {
        //std::cout << "Corrupted end < start. "<< std::endl;
        LOG_WARN_S << "Corrupted packet: Skipping " << end << " bytes because no start was found.";
        return -end;
    }

    if (start == 0)
    {
        if((size_t) end == buffer_size){
            //std::cout << "Found start of Packet, waiting end.. "<< std::endl;
            LOG_DEBUG_S << "Found start of Packet, waiting end.. "<< end;
            return 0;
        }
        else
        {
            //std::cout << "Found packet of type. "<< std::endl;
            LOG_DEBUG_S << "Found packet of type "<< buffer[1] << " and size "<< end;
            return end;
        }
    }

    int i;
    //Find full stream of values
    for(i = 0; i < start-1; i+=2)
        if((~buffer[i] | buffer[i+1]) & 0b10000000)
            break;

    if (i == 0)
    {
        LOG_WARN_S << "Corrupted packet: didn't find a one followed by a zero";
        return -1;
    }
    else
        return i;
}

int range_sensor_micro_epsilon::findFirstWord(const uint8_t *buffer, size_t buffer_size, const uint32_t *cmd, int start_at, size_t cmd_size){
    for(int i = start_at; i < (int)(buffer_size-cmd_size); i++ )
        if(getWord(&buffer[i]) == *cmd)
            return i+cmd_size;
    return buffer_size;
}
