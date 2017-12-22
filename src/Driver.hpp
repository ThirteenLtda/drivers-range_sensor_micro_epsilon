#ifndef _RANGESENSOR_RANGE_SENSOR_HPP_
#define _RANGESENSOR_RANGE_SENSOR_HPP_

#include <iodrivers_base/Driver.hpp>
#include  "PacketTypes.hpp"
#include <vector>



namespace range_sensor_micro_epsilon
{

    class RangeSensor: public iodrivers_base::Driver
    {
    private:
        uint8_t msg[LASER_PACKET_SIZE];
        ErrorStats stats;
        std::vector<double> range_value;
        double smr;//start measuring range
        double mr;//measuring range

    public:
        const ErrorStats& getErrors() const;
        double getStartMeasuringRange() const;
        double getMeasuringRange() const;
        /**
         * Basic RS422 communication for Laser Range Sensor.
         * Protocol ILD1402
         *
         */
        RangeSensor();
        ~RangeSensor();
        void openURI(const std::string &uri);

        std::vector<double>readRanges(int timeout);
        double rawToMeasurement(const uint8_t *buffer);
        double DVOToMeasurement(uint16_t dvo);
        uint16_t rawToDVO(const uint8_t *buffer);
        int extractPacket(const uint8_t *buffer, size_t buffer_size) const;
    };

} // end namespace range_sensor_micro_epsilon

#endif // _RANGESENSOR_RANGE_SENSOR_HPP_
