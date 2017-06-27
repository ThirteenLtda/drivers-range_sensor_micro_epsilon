#include "Driver.hpp"
#include "packet_types.hpp"
#include <iostream>
#include <cstring>
#include <base-logging/Logging.hpp>

using namespace std;
using namespace range_sensor_micro_epsilon;

static uint32_t getWord(const uint8_t* buffer){
    return be32toh(*(reinterpret_cast<const uint32_t*> (buffer)));
}

static int findFirstWord(uint8_t const buffer[], size_t buffer_size, uint32_t cmd, int start_at = 0){
    for(int i = start_at; i < (int)(buffer_size-4); i++ ) //4 is word byte size
        if(getWord(&buffer[i]) == cmd)
            return i+4;
    return buffer_size;
}

RangeSensor::RangeSensor():
    iodrivers_base::Driver(1024),range_value(0),smr(0.2),mr(0.6)
{
    stats.clear();
}

RangeSensor::~RangeSensor()
{
}

uint16_t RangeSensor::rawToDVO(uint8_t const buffer[]){
    return (buffer[0] & 0b01111111)*(1<<7) + buffer[1];
}

double RangeSensor::DVOToMeasurement(uint16_t dvo){
    return  ((static_cast<double>(dvo))*1.02/16368 - 0.01)*mr + smr;
}

double RangeSensor::rawToMeasurement(uint8_t const buffer[]){
    return  DVOToMeasurement(rawToDVO(buffer));
}

std::vector<double> RangeSensor::readPacket(int timeout)
{
    size_t size = Driver::readPacket(
            msg,
            LASER_PACKET_SIZE,
            base::Time::fromMilliseconds(timeout));

    range_value.clear();

    if(getWord(msg) == REPLY_START)
        return range_value;

    range_value.reserve(size/2);

    LOG_DEBUG_S << "reading packet with "<< size/2 << " measurements";

    for(size_t i = 0; i < size-1; i+=2)
        if((~msg[i] | msg[i+1]) & 0b10000000)
            throw std::runtime_error("extractPacket has extracted an invalid package!");
        else{
            uint16_t dvo = rawToDVO(&msg[i]);
            if(dvo > 16367)
                switch (dvo) {
                case 16370:
                    stats.no_object_detected += 1;
                    continue;
                case 16372:
                    stats.too_close_to_the_sensor += 1;
                    continue;
                case 16374:
                    stats.too_far_from_the_sensor += 1;
                    continue;
                case 16376:
                    stats.target_can_not_be_evaluated += 1;
                    continue;
                case 16380:
                    stats.target_moves_towards_the_sensor += 1;
                    continue;
                case 16382:
                    stats.target_moves_away_from_sensor += 1;
                    continue;
                default:
                    stats.unknown +=1;
                    continue;
                }

            range_value.push_back(DVOToMeasurement(dvo));
        }

    return range_value;
}

int RangeSensor::extractPacket(const uint8_t *buffer, size_t buffer_size) const
{

    if(buffer_size < 2) //throw std::runtime_error("Packet too small to contain range values.");
        return 0;


    int start = findFirstWord(buffer,buffer_size,REPLY_START);
    int end = findFirstWord(buffer,buffer_size,REPLY_END);

    if (end < start)
    {
        LOG_WARN_S << "Corrupted packet: Skipping " << end << " bytes because no start was found.";
        return -end;
    }

    if (start == 0)
    {
        if((size_t) end == buffer_size){
            LOG_DEBUG_S << "Found start of Packet, waiting end.. "<< end;
            return 0;
        }
        else
        {
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


void RangeSensor::openURI(const string &uri)
{
    //stats.clear();
    Driver::openURI(uri);
}
