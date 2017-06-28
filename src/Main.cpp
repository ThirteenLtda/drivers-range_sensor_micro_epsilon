#include <iostream>
#include <range_sensor_micro_epsilon/Driver.hpp>
#include  "PacketTypes.hpp"
#include <unistd.h>
#include <iomanip>


using namespace range_sensor_micro_epsilon;
int main(int argc, char** argv)
{
    RangeSensor sensor;
    sensor.openURI("serial:///dev/ttyUSB0:115200");

    const int tries =   20;
    for(int i=0;i<tries;i++) {

        try{
            std::vector<double> ranges = sensor.readRanges(1000);
            for (size_t index = 0; index != ranges.size(); ++index){
                std::cout << std::setfill('0') << std::setw(5) << std::dec << i << " | "
                          << index+1 << "/" << ranges.size() << " - ";
                std::cout << ranges[index] << std::endl;
            }
        }
        catch(const iodrivers_base::TimeoutError& e){
            std::cout << i+1 << "/" << tries <<")No Packet Received...\n" << e.what() << std::endl;
            sensor.close();

            std::cout << " closing and reopening" << std::endl;
            usleep(10000000);
            sensor.openURI("serial:///dev/ttyUSB0:115200");
            sensor.clear();

        }


        std::cout << "no object detected - " << sensor.getErrors().no_object_detected << "\n"
                  << "too close to the sensor - " << sensor.getErrors().too_close_to_the_sensor << "\n"
                  << "too far from the sensor - " << sensor.getErrors().too_far_from_the_sensor << "\n"
                  << "target can not be evaluated - " << sensor.getErrors().target_can_not_be_evaluated << "\n"
                  << "target moves towards the sensor - " << sensor.getErrors().target_moves_towards_the_sensor << "\n"
                  << "target moves away from sensor - " << sensor.getErrors().target_moves_away_from_sensor << "\n"
                  << "unknown - " << sensor.getErrors().unknown << std::endl;

        std::cout << "bad_rx = " << sensor.getStats().bad_rx << "\n"
                  << "good_rx = " << sensor.getStats().good_rx << "\n"
                  << "stamp = " << sensor.getStats().stamp << "\n"
                  << "tx = " << sensor.getStats().tx << "\n"
                  << "isValid = " << sensor.isValid() << "\n"
                  << "hasPacket = " << sensor.hasPacket() << "\n"
                  << "Fd = " << sensor.getFileDescriptor() << std::endl;

        usleep(100000);

    }

    sensor.close();

    return 0;
}
