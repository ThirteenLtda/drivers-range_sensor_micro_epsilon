#include <iostream>
#include <range_sensor_micro_epsilon/Driver.hpp>
#include <unistd.h>
#include <iomanip>


using namespace range_sensor_micro_epsilon;
int main(int argc, char** argv)
{
    std::string port;
    RangeSensor sensor;
    //std::cout << "Enter USB port: ";
    //std::cin >> port;
    sensor.openURI("serial:///dev/ttyUSB0:115200");

    const int tries =   20;
    for(int i=0;i<tries;i++) {

        try{
            sensor.readPacket(1000);
            for (size_t index = 0; index != sensor.getRange().size(); ++index){
                std::cout << std::setfill('0') << std::setw(5) << std::dec << i << " | "
                          << index+1 << "/" << sensor.getRange().size() << " - ";
                std::cout << sensor.getRange()[index] << std::endl;
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


        usleep(100000);

    }

    sensor.close();

    return 0;
}
