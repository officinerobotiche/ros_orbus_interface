#ifndef UNAVCONTROLLER_H
#define UNAVCONTROLLER_H

#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>

#include "hardware/GenericController.h"

namespace ORController
{

class uNavController : public GenericController, public hardware_interface::RobotHW
{
public:
    uNavController(orbus::serial_controller *serial);

    void updateJointsFromHardware();
    void writeCommandsToHardware(ros::Duration period);

private:
    void motorFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);
};

}

#endif // UNAVCONTROLLER_H
