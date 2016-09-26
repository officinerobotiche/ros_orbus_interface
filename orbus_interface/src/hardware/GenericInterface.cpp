#include "hardware/GenericInterface.h"

namespace ORInterface
{

GenericInterface::GenericInterface(const ros::NodeHandle &nh, orbus::serial_controller *serial) : mNh(nh), mSerial(serial)
{
    bool initCallback = mSerial->addCallback(&GenericInterface::systemFrame, this, HASHMAP_SYSTEM);
}

void GenericInterface::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
}

void GenericInterface::systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    ROS_INFO_STREAM("Generic Controller I'm here");
}

}
