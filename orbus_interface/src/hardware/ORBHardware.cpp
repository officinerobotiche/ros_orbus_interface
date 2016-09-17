/**
*
*  \author     Raffaello Bonghi <raffaello.bonghi@officinerobotiche.it>
*  \copyright  Copyright (c) 2014-2015, Officine Robotiche, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to developers@officinerobotiche.it
*
*/

#include "hardware/ORBHardware.h"

using namespace std;

#define NUMBER_PUB 10

ORBHardware::ORBHardware(const ros::NodeHandle& nh, const ros::NodeHandle &private_nh, SerialController* serial, double frequency)
: nh_(nh), private_nh_(private_nh), serial_(serial), name_board_("Nothing"), type_board_("Nothing") {

    serial_->addCallback(&ORBHardware::defaultPacket, this);
    //serial_->addErrorCallback(&ORBHardware::errorPacket, this);

    //Timer
    timer_ = nh_.createTimer(ros::Duration(1 / frequency), &ORBHardware::timerCallback, this, false, false);
    timer_.start();
}

ORBHardware::~ORBHardware() {
    serial_->clearCallback();
    //serial_->clearErrorCallback();
    timer_.stop();
}

void ORBHardware::timerCallback(const ros::TimerEvent& event) {
    /// Send all messages
    // ROS_INFO_STREAM("Number of Packages: " << list_send_.size());
    //Take a copy of all messages to send in serial
    try {
        serial_->sendList();
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void ORBHardware::initializeDiagnostics() {
    diagnostic_updater_.setHardwareID(name_board_);
}

/**
* External hook to trigger diagnostic update
*/
void ORBHardware::updateDiagnostics() {
    diagnostic_updater_.force_update();
}

void ORBHardware::loadParameter() {
    // Build a list of packets
    vector<packet_information_t> list_packet;

    //Add other parameter request
    if (callback_add_parameter)
        callback_add_parameter(&list_packet);

    try {
        serial_->parserSendPacket(list_packet, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    //addPacketSend(list_packet);
}

void ORBHardware::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
}

//void ORBHardware::errorPacket(const unsigned char& command, const message_abstract_u* packet) {
//    ROS_ERROR("Error in Packet: %d", command);
//}

void ORBHardware::defaultPacket(const unsigned char& command, const message_abstract_u* packet) {
    ROS_INFO("Default Packet: %d", command);
}

void ORBHardware::addParameterPacketRequest(const boost::function<void (std::vector<packet_information_t>*) >& callback) {
    callback_add_parameter = callback;
}

void ORBHardware::clearParameterPacketRequest() {
    callback_add_parameter.clear();
}

