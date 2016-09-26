#ifndef GENERICCONTROLLER_H
#define GENERICCONTROLLER_H

#include <ros/ros.h>
#include "hardware/serial_controller.h"

namespace ORController
{

class GenericController
{
public:
    GenericController(const ros::NodeHandle &nh, orbus::serial_controller *serial);

protected:
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    // Serial controller communication
    orbus::serial_controller *mSerial;
private:
    void systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

};

}

#endif // GENERICCONTROLLER_H
