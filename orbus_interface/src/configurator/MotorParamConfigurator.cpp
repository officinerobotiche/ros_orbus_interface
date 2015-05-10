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

#include "configurator/MotorParamConfigurator.h"

using namespace std;

MotorParamConfigurator::MotorParamConfigurator(const ros::NodeHandle &nh, std::string name, unsigned int number, ParserPacket *serial)
    : nh_(nh), serial_(serial)
{
    //Namespace
    name_ = name;// + "/param";
    // Set command message
    command_.bitset.motor = number;
    command_.bitset.command = MOTOR_PARAMETER;
    // Set message to frequency information
    last_frequency_.hashmap = HASHMAP_MOTOR;
    last_frequency_.number = 0; ///< TODO To correct

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
                        /// Set paramater
                        setParam(packet.message.motor_parameter);
                    }
                }
                break;
            }
        }
    } else {
        /// Send configuration to board
        sendToSerial(getParam());
    }

    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavParameterConfig>(ros::NodeHandle("~" + name_));
    dynamic_reconfigure::Server<orbus_interface::UnavParameterConfig>::CallbackType cb = boost::bind(&MotorParamConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}


void MotorParamConfigurator::setParam(motor_parameter_t parameter) {
    motor_parameter_encoder_t encoder = parameter.encoder;
    motor_parameter_bridge_t bridge = parameter.bridge;
    nh_.setParam(name_ + "/CPR", ((int) encoder.cpr));
    nh_.setParam(name_ + "/Ratio", (double) parameter.ratio);
    /// Convert from mV in V
    nh_.setParam(name_ + "/Bridge", ((double)(bridge.volt)/1000));
    nh_.setParam(name_ + "/Encoder", encoder.position);
    nh_.setParam(name_ + "/Rotation", parameter.rotation);
    nh_.setParam(name_ + "/Enable", bridge.enable);
}

motor_parameter_t MotorParamConfigurator::getParam() {
    motor_parameter_t parameter;
    int temp_int;
    double temp_double;
    nh_.getParam(name_ + "/CPR", temp_int);
    parameter.encoder.cpr = (uint16_t) temp_int;
    nh_.getParam(name_ + "/Ratio", temp_double);
    parameter.ratio = (float) temp_double;
    /// Convert from mV in V
    nh_.getParam(name_ + "/Bridge", temp_double);
    parameter.bridge.volt = (uint16_t) temp_double*1000;
    nh_.getParam(name_ + "/Encoder", temp_int);
    parameter.encoder.position = (uint8_t) temp_int;
    nh_.getParam(name_ + "/Rotation", temp_int);
    parameter.rotation = (int8_t) temp_int;
    nh_.getParam(name_ + "/Enable", temp_int);
    parameter.bridge.enable = (uint8_t) temp_int;
    return parameter;
}

void MotorParamConfigurator::sendToSerial(motor_parameter_t parameter) {
    packet_t packet_send = serial_->encoder(serial_->createDataPacket(command_.command_message, HASHMAP_MOTOR, (message_abstract_u*) & parameter));
    try {
        serial_->sendSyncPacket(packet_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void MotorParamConfigurator::reconfigureCB(orbus_interface::UnavParameterConfig &config, uint32_t level) {

    motor_parameter_t param;
    param.encoder.cpr = (uint16_t) config.CPR;
    param.bridge.enable = (uint8_t) config.Enable;
    param.encoder.position = (uint8_t) config.Encoder;
    param.ratio = (float) config.Ratio;
    param.rotation = (int8_t) config.Rotation;
    param.bridge.volt = (int16_t) (config.Bridge*1000);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_param_ = param;
      default_param_ = last_param_;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      param = default_param_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }
    /// Send to serial
    sendToSerial(param);
    last_param_ = param;
}
