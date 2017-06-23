#ifndef _RANGESENSOR_RANGE_SENSOR_HPP_
#define _RANGESENSOR_RANGE_SENSOR_HPP_

#include <iodrivers_base/Driver.hpp>
#include <vector>



namespace range_sensor_micro_epsilon
{
    int findFirstWord(const uint8_t *buffer, size_t buffer_size, const uint32_t *cmd, int start_at=0, size_t cmd_size = 4);

    class RangeSensor: public iodrivers_base::Driver
    {
    private:
        uint8_t msg[1024];
        uint16_t dvalue;
        std::vector<float> range_value;

    public:
        const std::vector<float> &getRange() const{
            return range_value;
        }

        float smr;//start measuring range
        float mr;//measuring range
        /**
         * Basic RS422 communication for Laser Range Sensor.
         * Protocol ILD1402
         *
         */
        RangeSensor();
        ~RangeSensor();

        void read();
        double rawToMeasurement(const uint8_t* buffer);
        double DVOToMeasurement(uint16_t dvo);
        uint16_t rawToDVO(const uint8_t* buffer);
        bool readPacket(int timeout);
        int extractPacket(const uint8_t *buffer, size_t buffer_size) const;

    };

} // end namespace range_sensor_micro_epsilon

#endif // _RANGESENSOR_RANGE_SENSOR_HPP_
