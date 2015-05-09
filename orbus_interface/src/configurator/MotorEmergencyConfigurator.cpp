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
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "configurator/MotorEmergencyConfigurator.h"

using namespace std;

MotorEmergencyConfigurator::MotorEmergencyConfigurator(const ros::NodeHandle& nh, std::string name, unsigned int number, ParserPacket *serial)
    : nh_(nh), serial_(serial)
{
    //Namespace
    name_ = name + "/emergency";
    // Set command message
    command_.bitset.motor = number;
    command_.bitset.command = MOTOR_EMERGENCY;

    /// Check existence namespace otherwise get information from board
    if (!nh_.hasParam(name_)) {
        /// Build a list of messages
        std::vector<packet_information_t> list_send;
        list_send.push_back(serial->createPacket(command_.command_message, PACKET_REQUEST, HASHMAP_MOTOR));

        /// Build a packet
        packet_t packet = serial->encoder(list_send);
        /// Receive information
        vector<packet_information_t> receive = serial->parsing(serial->sendSyncPacket(packet));
        /// Decode all messages
        for (vector<packet_information_t>::iterator list_iter = receive.begin(); list_iter != receive.end(); ++list_iter) {
            packet_information_t packet = (*list_iter);
            switch (packet.option) {
            case PACKET_NACK:
                ///< Send a message ERROR
                break;
            case PACKET_DATA:
                if (packet.type == HASHMAP_MOTOR) {
                    if(packet.command = command_.command_message) {
                        nh_.setParam(name_ + "/Slope_time", packet.message.motor_pid.kp);
                        nh_.setParam(name_ + "/Bridge_off", packet.message.motor_pid.ki);
                        nh_.setParam(name_ + "/Timeout", packet.message.motor_pid.kd);
                    }
                }
                break;
            }
        }
    }

    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavEmergencyConfig>(ros::NodeHandle("~" + name_));
    dynamic_reconfigure::Server<orbus_interface::UnavEmergencyConfig>::CallbackType cb = boost::bind(&MotorEmergencyConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void MotorEmergencyConfigurator::reconfigureCB(orbus_interface::UnavEmergencyConfig &config, uint32_t level) {

    motor_emergency_t emergency;
    emergency.bridge_off = (float) config.Bridge_off;
    emergency.slope_time = (float) config.Slope_time;
    emergency.timeout = (uint16_t) config.Timeout;

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_emergency_ = emergency;
      default_emer_ = last_emergency_;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      emergency = default_emer_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    packet_t packet_send = serial_->encoder(serial_->createDataPacket(command_.command_message, HASHMAP_MOTOR, (message_abstract_u*) & emergency));

    try {
        serial_->sendSyncPacket(packet_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    last_emergency_ = emergency;
}
