#ifndef _RANGESENSOR_RANGE_SENSOR_HPP_
#define _RANGESENSOR_RANGE_SENSOR_HPP_

#include <iodrivers_base/Driver.hpp>
#include <vector>



namespace range_sensor_micro_epsilon
{
    int find_last(const uint8_t *buffer, size_t buffer_size, const uint32_t *cmd, int start_at=-1, size_t cmd_size = 4);
    int find_first(const uint8_t *buffer, size_t buffer_size, const uint32_t *cmd, int start_at=0, size_t cmd_size = 4);

    class RangeSensor: public iodrivers_base::Driver
    {
    private:
        uint8_t msg[1000];
        uint16_t dvalue;
        std::vector<float> range_value;

    public:
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
        void openSerial(std::string const& port, int baudrate = 115200);
        float measurementILD1402(const uint8_t* dvo);
        void close();
        bool readPacket(int timeout);
        int extractPacket(const uint8_t *buffer, size_t buffer_size) const;

    };

} // end namespace range_sensor_micro_epsilon

#endif // _RANGESENSOR_RANGE_SENSOR_HPP_
