#include "hardware/GenericInterface.h"

namespace ORInterface
{

GenericInterface::GenericInterface(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, orbus::serial_controller *serial)
    : mNh(nh)
    , private_mNh(private_nh)
    , mSerial(serial)
    , serial_status(true)
{
    bool initCallback = mSerial->addCallback(&GenericInterface::systemFrame, this, HASHMAP_SYSTEM);

    name_board = "Unknown";
}

void GenericInterface::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
}

void GenericInterface::systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    ROS_INFO_STREAM("Generic Controller I'm here");
}

}
