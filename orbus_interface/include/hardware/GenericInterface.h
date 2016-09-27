#ifndef GENERICCONTROLLER_H
#define GENERICCONTROLLER_H

#include <ros/ros.h>
#include "hardware/serial_controller.h"

namespace ORInterface
{

class GenericInterface
{
public:
    GenericInterface(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, orbus::serial_controller *serial);

protected:
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    ros::NodeHandle private_mNh;
    // Serial controller communication
    orbus::serial_controller *mSerial;

    //Name of device
    string name_board;
private:
    void systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

};

}

#endif // GENERICCONTROLLER_H
