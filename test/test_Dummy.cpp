#include <boost/test/unit_test.hpp>
#include <range_sensor_micro_epsilon/Dummy.hpp>

using namespace range_sensor_micro_epsilon;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    range_sensor_micro_epsilon::DummyClass dummy;
    dummy.welcome();
}
