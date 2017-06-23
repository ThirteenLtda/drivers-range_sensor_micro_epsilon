#include "Driver.hpp"
#include "packet_types.hpp"
#include <iostream>
#include <cstring>
#include <base-logging/Logging.hpp>

using namespace std;
using namespace range_sensor_micro_epsilon;

inline uint32_t get_word(const uint8_t* buffer){
    return be32toh(*(reinterpret_cast<const uint32_t*> (buffer)));
}

RangeSensor::RangeSensor():
    iodrivers_base::Driver(1024),range_value(0),smr(0.2),mr(0.6)
{
}

RangeSensor::~RangeSensor()
{
}

void RangeSensor::openSerial(std::string const& port, int baudrate)
{
    LOG_DEBUG_S << "opening serial port: " << port << " with baudrate: " << baudrate ;
    Driver::openSerial(port,baudrate);
}

void RangeSensor::close()
{
    LOG_DEBUG_S <<"closing serial port: " ;
    Driver::close();
}

uint16_t RangeSensor::raw_dvo(const uint8_t* buffer){
    return (buffer[0] & 0b01111111)*(1<<7) + buffer[1];
}

float RangeSensor::measurementILD1402(uint16_t dvo){
    return  (((float)dvo)*1.02/16368 - 0.01)*mr + smr;
}

float RangeSensor::measurementILD1402(const uint8_t* buffer){
    return  measurementILD1402(raw_dvo(buffer));
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
            uint16_t dvo = raw_dvo(&msg[i]);
            if(dvo > 16367)
                continue;
            range_value.push_back(measurementILD1402(dvo));
        }

    if(range_value.size() > 0)
        return true;

    return false;
}

int RangeSensor::extractPacket(const uint8_t *buffer, size_t buffer_size) const
{
    if(buffer_size < 2){ //throw std::runtime_error("Packet too small to contain range values.");
        //std::cout << "Packet too small to contain range values. "<< std::endl;
        return 0;
    }

    int start = find_first(buffer,buffer_size,&REPLY_START);
    int end = find_first(buffer,buffer_size,&REPLY_END);

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

int range_sensor_micro_epsilon::find_first(const uint8_t *buffer, size_t buffer_size, const uint32_t *cmd, int start_at, size_t cmd_size){
    for(int i = start_at; i < (int)(buffer_size-cmd_size); i++ )
        if(get_word(&buffer[i]) == *cmd)
            return i+cmd_size;
    return buffer_size;
}

int range_sensor_micro_epsilon::find_last(const uint8_t *buffer, size_t buffer_size, const uint32_t *cmd, int start_at, size_t cmd_size){
    if(start_at == 1)
        start_at = buffer_size-cmd_size;
    for(size_t i = start_at; i >= 0; i-- )
        if(memcmp(&buffer[i],cmd,cmd_size) == 0)
            return i+cmd_size;
    return -1;
}

/*
 *


    int start = find_last(buffer,buffer_size,&REPLY_START);
    int end = find_last(buffer,buffer_size,&REPLY_END);
    if( end < start )
        throw std::runtime_error("Packet size to small or corrupted, do not contain message end.");
    //if( end == -1 && start == -1)
    //    end = 0;
    if(end >= (int)buffer_size -2)
        throw std::runtime_error("Packet does not contain trailling range values.");

    if( (~buffer[buffer_size-1] & 0b10000000) && (buffer[buffer_size-2] & 0b10000000))
    {

        return 2;
    }

    if( (~buffer[buffer_size-2] & 0b10000000) && (buffer[buffer_size-3] & 0b10000000))
    {
        dvalue = (buffer[buffer_size-2] & 0b01111111)*(1<<7) + buffer[buffer_size-3];
        range_value =  (((float)dvalue)*1.02/16368 - 0.01)*mr + smr;
        return 2;
    }
*/