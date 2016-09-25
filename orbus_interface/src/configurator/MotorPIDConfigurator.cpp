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
*     * Neither the name of Officine Robotiche, Inc. nor the
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

#include "configurator/MotorPIDConfigurator.h"

using namespace std;

MotorPIDConfigurator::MotorPIDConfigurator(const ros::NodeHandle& nh, SerialController *serial, std::string path, std::string name, unsigned int number, unsigned int type)
    : nh_(nh), serial_(serial), name_(path + "/pid/" + name)
{
    //Namespace
    //name_ = path + "/pid/" + name;
    // Set command message
    command_.bitset.motor = number;
    command_.bitset.command = type;
    //ROS_INFO_STREAM("Configurator: "<< name << " command: " << (int) command_.command_message);


    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavPIDConfig>(ros::NodeHandle("~" + name_));
    dynamic_reconfigure::Server<orbus_interface::UnavPIDConfig>::CallbackType cb = boost::bind(&MotorPIDConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void MotorPIDConfigurator::initConfigurator() {
    /// Check existence namespace otherwise get information from board
    if (!nh_.hasParam(name_)) {
        //ROS_INFO_STREAM("SEND Request for: " << name_);
        serial_->addPacketSend(serial_->createPacket(command_.command_message, PACKET_REQUEST, HASHMAP_MOTOR));
    } else {
        //ROS_INFO_STREAM("GET param from " <<  name_ << " and send");
        /// Send configuration to board
        motor_pid_t parameter = getParam();
        serial_->addPacketSend(serial_->createDataPacket(command_.command_message, HASHMAP_MOTOR, (message_abstract_u*) & parameter));
    }
}

void MotorPIDConfigurator::setParam(motor_pid_t parameter) {
    nh_.setParam(name_ + "/Kp", parameter.kp);
    nh_.setParam(name_ + "/Ki", parameter.ki);
    nh_.setParam(name_ + "/Kd", parameter.kd);
    nh_.setParam(name_ + "/Frequency", (int) parameter.frequency);
    nh_.setParam(name_ + "/Enable", (parameter.enable > 0 ? true : false));
}

motor_pid_t MotorPIDConfigurator::getParam() {
    motor_pid_t pid;

    double temp_double;
    int temp_int;
    bool temp;
    nh_.getParam(name_ + "/Kp", temp_double);
    pid.kp = (float) temp_double;
    nh_.getParam(name_ + "/Ki", temp_double);
    pid.ki = (float) temp_double;
    nh_.getParam(name_ + "/Kd", temp_double);
    pid.kd = (float) temp_double;
    nh_.getParam(name_ + "/Frequency", temp_int);
    pid.frequency = (uint32_t) temp_int;
    nh_.getParam(name_ + "/Enable", temp);
    pid.enable = (uint8_t) temp;

    return pid;
}

void MotorPIDConfigurator::reconfigureCB(orbus_interface::UnavPIDConfig &config, uint32_t level) {

    motor_pid_t pid;
    pid.kp = (float) config.Kp;
    pid.ki = (float) config.Ki;
    pid.kd = (float) config.Kd;
    pid.frequency = (uint32_t) config.Frequency;
    pid.enable = (uint8_t) config.Enable;

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_pid_ = pid;
      default_pid_ = last_pid_;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      pid = default_pid_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    std::vector<packet_information_t> list_send;
    if(last_pid_.kp != pid.kp || last_pid_.ki != pid.ki || last_pid_.kd != pid.kd) {
        list_send.push_back(serial_->createDataPacket(command_.command_message, HASHMAP_MOTOR, (message_abstract_u*) & pid));
        last_pid_ = pid;
        config.Kp = pid.kp;
        config.Ki = pid.ki;
        config.Kd = pid.kd;
        config.Frequency = pid.frequency;
        config.Enable = pid.enable;
    }

    /// Send to serial
    serial_->addPacketSend(serial_->createDataPacket(command_.command_message, HASHMAP_MOTOR, (message_abstract_u*) & pid));
}
