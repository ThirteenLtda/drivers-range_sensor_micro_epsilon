#include <boost/test/unit_test.hpp>
#include <range_sensor_micro_epsilon/packet_types.hpp>
#include <range_sensor_micro_epsilon/Driver.hpp>

using namespace range_sensor_micro_epsilon;

BOOST_AUTO_TEST_CASE(convert_two_bytes_to_double_distance)
{
    RangeSensor laser;
    const uint8_t buffer[2] = {0b11010111, 0b00111000};
    double dist = 0.68744868 * laser.getMeasuringRange() + laser.getStartMeasuringRange();
    BOOST_CHECK_CLOSE(dist, laser.rawToMeasurement(buffer), 0.0001);
}

BOOST_AUTO_TEST_CASE(wrong_first_byte)
{
    RangeSensor laser;
    const uint8_t buffer[] = {0b01010111, 0b00111000,
                               0b10010111, 0b00111010,
                               0b11010011, 0b00111100,
                               0b11000111, 0b00101000,
                               0b11011111};
    BOOST_CHECK_EQUAL(-1, laser.extractPacket(buffer,9));
}

BOOST_AUTO_TEST_CASE(wrong_first_byte_with_start)
{
    RangeSensor laser;
    const uint8_t buffer[] = { 0b00111000,
                               0b10010111, 0b00111010,
                               0b11010011, 0b00111100,
                               0x49, 0x4C, 0x44, 0x31,
                              0b1101001, 0b00100100};
    BOOST_CHECK_EQUAL(-1, laser.extractPacket(buffer,11));
}

BOOST_AUTO_TEST_CASE(wrong_first_byte_with_lost_end)
{
    RangeSensor laser;
    const uint8_t buffer[] = { 0b00111000,
                                0b00111010,
                               0b11010011, 0b00111100,
                               0x20, 0x20, 0x0D, 0x0A,
                              0b1101001, 0b10010111,
                               0b00100100};
    BOOST_CHECK_EQUAL(-8, laser.extractPacket(buffer,11));
}

BOOST_AUTO_TEST_CASE(incomplete_stream_of_measurements)
{
    RangeSensor laser;
    const uint8_t buffer[] = {0b10010111, 0b00111000,
                               0b10010111, 0b00111010,
                               0b11010011, 0b00111100,
                               0b11000111, 0b00101000,
                               0b11011111};
    BOOST_CHECK_EQUAL(8, laser.extractPacket(buffer,9));
}

BOOST_AUTO_TEST_CASE(stream_of_measurements_ending_with_bad_bit)
{
    RangeSensor laser;
    const uint8_t buffer[] = {0b10010111, 0b00111000,
                               0b10010111, 0b00111010,
                               0b11010011, 0b00111100,
                               0b11000111, 0b10101000,
                               0b11011111, 0b00101110};
    BOOST_CHECK_EQUAL(6, laser.extractPacket(buffer,10));
}

BOOST_AUTO_TEST_CASE(stream_of_measurements_with_two_leading_ones)
{
    RangeSensor laser;
    const uint8_t buffer[] = {0b10010111, 0b10111000, 0b00111000};
    BOOST_CHECK_EQUAL(-1, laser.extractPacket(buffer,3));
}

BOOST_AUTO_TEST_CASE(complete_stream_of_measurements)
{
    RangeSensor laser;
    const uint8_t buffer[] = {0b10010111, 0b00111000,
                               0b10010111, 0b00111010,
                               0b11010011, 0b00111100,
                               0b11000111, 0b00101000,
                               0b11011111, 0b00101110};
    BOOST_CHECK_EQUAL(10, laser.extractPacket(buffer,10));
}
