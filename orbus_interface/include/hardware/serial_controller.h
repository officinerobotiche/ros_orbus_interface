#ifndef SERIAL_CONTROLLER_H
#define SERIAL_CONTROLLER_H

#include <ros/ros.h>
#include <serial/serial.h>

using namespace std;

namespace orbus
{

class serial_controller
{
public:
    serial_controller();

private:
    // Serial port object
    serial::Serial mSerial;
    // Serial port name
    string mSerialPort;
};

}

#endif // SERIAL_CONTROLLER_H
