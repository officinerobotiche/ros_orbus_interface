#include "hardware/GenericController.h"

GenericController::GenericController(orbus::serial_controller *serial) : mSerial(serial)
{
    bool initCallback = mSerial->addCallback(&GenericController::systemFrame, this, HASHMAP_SYSTEM);
}

void GenericController::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
}

void GenericController::systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    ROS_INFO_STREAM("Generic Controller I'm here");
}
