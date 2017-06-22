#include <iostream>
#include <range_sensor_micro_epsilon/range_sensor.hpp>
#include <unistd.h>
#include <iomanip>


using namespace range_sensor_micro_epsilon;
int main(int argc, char** argv)
{
    std::string port;
    RangeSensor sensor;
    std::cout << "Enter USB port: ";
    std::cin >> port;
    sensor.openSerial(port);

    for(int i=0;i<20000;i++) {
        if(sensor.readPacket(1000))
            for (size_t index = 0; index != sensor.getRange().size(); ++index){
                std::cout << std::setfill('0') << std::setw(5) << std::dec << i << " | "
                          << index+1 << "/" << sensor.getRange().size() << " - ";
                std::cout << sensor.getRange()[index] << std::endl;
            }
        else
            std::cout << "No Packet Received..." << std::endl;

        usleep(20000);

    }

    sensor.close();

    return 0;
}
