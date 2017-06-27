#ifndef _RANGESENSOR_RANGE_SENSOR_HPP_
#define _RANGESENSOR_RANGE_SENSOR_HPP_

#include <iodrivers_base/Driver.hpp>
#include <vector>



namespace range_sensor_micro_epsilon
{

    class RangeSensor: public iodrivers_base::Driver
    {
    private:
        uint8_t msg[1024];
        uint16_t dvalue;
        std::vector<double> range_value;

    public:
        const std::vector<double> &getRange() const{
            return range_value;
        }

        double smr;//start measuring range
        double mr;//measuring range
        /**
         * Basic RS422 communication for Laser Range Sensor.
         * Protocol ILD1402
         *
         */
        RangeSensor();
        ~RangeSensor();

        void read();
        double rawToMeasurement(const uint8_t buffer[]);
        double DVOToMeasurement(uint16_t dvo);

        uint16_t rawToDVO(const uint8_t buffer[]);
        std::vector<double> readPacket(int timeout);
        int extractPacket(const uint8_t *buffer, size_t buffer_size) const;

    };

} // end namespace range_sensor_micro_epsilon

#endif // _RANGESENSOR_RANGE_SENSOR_HPP_
