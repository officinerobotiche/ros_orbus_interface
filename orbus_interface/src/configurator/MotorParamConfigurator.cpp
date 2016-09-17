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

MotorParamConfigurator::MotorParamConfigurator(const ros::NodeHandle &nh, SerialController *serial, std::string name, unsigned int number)
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
        serial_->addPacketSend(serial_->createPacket(command_.command_message, PACKET_REQUEST, HASHMAP_MOTOR));
    } else {
        /// Send configuration to board
        motor_parameter_t parameter = getParam();
        serial_->addPacketSend(serial_->createDataPacket(command_.command_message, HASHMAP_MOTOR, (message_abstract_u*) & parameter));
    }

    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavParameterConfig>(ros::NodeHandle("~" + name_));
    dynamic_reconfigure::Server<orbus_interface::UnavParameterConfig>::CallbackType cb = boost::bind(&MotorParamConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}


void MotorParamConfigurator::setParam(motor_parameter_t parameter) {
    motor_parameter_encoder_t encoder = parameter.encoder;
    motor_parameter_bridge_t bridge = parameter.bridge;
    nh_.setParam(name_ + "/Ratio", (double) parameter.ratio);
    nh_.setParam(name_ + "/Rotation", (int) parameter.rotation);

    nh_.setParam(name_ + "/Bridge/Enable", (int) bridge.enable);
    nh_.setParam(name_ + "/Bridge/PWM_dead_zone", (int) bridge.pwm_dead_zone);
    nh_.setParam(name_ + "/Bridge/PWM_frequency", (int) bridge.pwm_frequency);
    nh_.setParam(name_ + "/Bridge/Volt_gain", (double) bridge.volt_gain);
    nh_.setParam(name_ + "/Bridge/Current_offset", (double) bridge.current_offset);
    nh_.setParam(name_ + "/Bridge/Current_gain", (double) bridge.current_gain);

    nh_.setParam(name_ + "/Encoder/CPR", (int) encoder.cpr);
    nh_.setParam(name_ + "/Encoder/Position", (int) encoder.type.position);
    nh_.setParam(name_ + "/Encoder/Z_index", (int) encoder.type.z_index);
    nh_.setParam(name_ + "/Encoder/Channels", (int) (encoder.type.channels + 1));
}

motor_parameter_t MotorParamConfigurator::getParam() {
    motor_parameter_t parameter;
    int temp_int;
    double temp_double;
    nh_.getParam(name_ + "/Ratio", temp_double);
    parameter.ratio = (float) temp_double;
    nh_.getParam(name_ + "/Rotation", temp_int);
    parameter.rotation = (int8_t) temp_int;

    nh_.getParam(name_ + "/Bridge/Enable", temp_int);
    parameter.bridge.enable = (uint8_t) temp_int;
    nh_.getParam(name_ + "/Bridge/PWM_dead_zone", temp_int);
    parameter.bridge.pwm_dead_zone = (uint16_t) temp_int;
    nh_.getParam(name_ + "/Bridge/PWM_requency", temp_int);
    parameter.bridge.pwm_frequency = (uint16_t) temp_int;
    nh_.getParam(name_ + "/Bridge/Volt_gain", temp_double);
    parameter.bridge.volt_gain = (float) temp_double;
    nh_.getParam(name_ + "/Bridge/Current_offset", temp_double);
    parameter.bridge.current_offset = (float) temp_double;
    nh_.getParam(name_ + "/Bridge/Current_gain", temp_double);
    parameter.bridge.current_gain = (float) temp_double;

    nh_.getParam(name_ + "/Encoder/CPR", temp_int);
    parameter.encoder.cpr = (uint16_t) temp_int;
    nh_.getParam(name_ + "/Encoder/Position", temp_int);
    parameter.encoder.type.position = (uint8_t) temp_int;
    nh_.getParam(name_ + "/Encoder/Z_index", temp_int);
    parameter.encoder.type.z_index = (uint8_t) temp_int;
    nh_.getParam(name_ + "/Encoder/Channels", temp_int);
    parameter.encoder.type.channels = (uint8_t) (temp_int - 1);

    return parameter;
}

void MotorParamConfigurator::reconfigureCB(orbus_interface::UnavParameterConfig &config, uint32_t level) {

    motor_parameter_t parameter;
    parameter.ratio = (float) config.Ratio;
    parameter.rotation = (int8_t) config.Rotation;

    parameter.bridge.enable = (uint8_t) config.Enable;
    parameter.bridge.pwm_dead_zone = (uint16_t) config.PWM_Dead_zone;
    parameter.bridge.pwm_frequency = (uint16_t) config.PWM_Frequency;
    parameter.bridge.volt_gain = (float) config.Voltage_Gain;
    parameter.bridge.current_offset = (float) config.Current_Offset;
    parameter.bridge.current_gain = (float) config.Current_Gain;

    parameter.encoder.cpr = (uint16_t) config.CPR;
    parameter.encoder.type.position = (uint8_t) config.Encoder_Position;
    parameter.encoder.type.z_index = (uint8_t) config.Encoder_Z_index;
    parameter.encoder.type.channels = (uint8_t) (config.Encoder_Channels - 1);



    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_param_ = parameter;
      default_param_ = last_param_;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      parameter = default_param_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    /// Send to serial
    serial_->addPacketSend(serial_->createDataPacket(command_.command_message, HASHMAP_MOTOR, (message_abstract_u*) & parameter));

    last_param_ = parameter;
}
