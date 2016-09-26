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
* Please send comments, questions, or patches to developer@officinerobotiche.it
*
*/

#include "configurator/MotorPIDConfigurator.h"

using namespace std;

MotorPIDConfigurator::MotorPIDConfigurator(const ros::NodeHandle& nh, orbus::serial_controller *serial, string path, std::string name, unsigned int type, unsigned int number)
    : nh_(nh)
    , mSerial(serial)
{
    //Namespace
    mName = path + "/pid/" + name;
    // Set command message
    mCommand.bitset.motor = number;
    mCommand.bitset.command = type;
    ROS_INFO_STREAM("Configurator: "<< mName << " command: " << number);

    /// Check existence namespace otherwise get information from board
    if (!nh_.hasParam(mName)) {
        ROS_WARN_STREAM("Any PID paramerter for: " << mName << " . Send request to uNav");
        // Build a packet
        packet_information_t frame = CREATE_PACKET_RESPONSE(mCommand.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
        // Add packet in the frame
        if(mSerial->addFrame(frame)->sendList())
        {
            ROS_WARN_STREAM("Write PARAM from uNav");
        }
    } else {
        ROS_INFO_STREAM("GET param from " <<  mName << " and send");
        /// Send configuration to board
        motor_pid_t parameter = getParam();
       // mSerial->addPacketSend(mSerial->createDataPacket(mCommand.command_message, HASHMAP_MOTOR, (message_abstract_u*) & parameter));
    }

    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavPIDConfig>(ros::NodeHandle("~" + mName));
    dynamic_reconfigure::Server<orbus_interface::UnavPIDConfig>::CallbackType cb = boost::bind(&MotorPIDConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void MotorPIDConfigurator::setParam(motor_pid_t parameter) {
    bool enable = (parameter.enable > 0 ? true : false);
    ROS_INFO_STREAM("Parameter [Kp:" << parameter.kp << ", Ki:" << parameter.ki << ", Kd:" << parameter.kd << ", Freq:" << parameter.frequency << "Hz, En:" << enable << "]");
    //nh_.setParam(mName + "/Kp", ((double) parameter.kp));
//    nh_.setParam(mName + "/Ki", (double) parameter.ki);
//    nh_.setParam(mName + "/Kd", (double) parameter.kd);
//    nh_.setParam(mName + "/Frequency", (int) parameter.frequency);
//    nh_.setParam(mName + "/Enable", (parameter.enable > 0 ? true : false));
}

motor_pid_t MotorPIDConfigurator::getParam() {
    motor_pid_t pid;

    double temp_double;
    int temp_int;
    bool temp;
    nh_.getParam(mName + "/Kp", temp_double);
    pid.kp = (float) temp_double;
    nh_.getParam(mName + "/Ki", temp_double);
    pid.ki = (float) temp_double;
    nh_.getParam(mName + "/Kd", temp_double);
    pid.kd = (float) temp_double;
    nh_.getParam(mName + "/Frequency", temp_int);
    pid.frequency = (uint32_t) temp_int;
    nh_.getParam(mName + "/Enable", temp);
    pid.enable = (uint8_t) temp;

    return pid;
}

void MotorPIDConfigurator::reconfigureCB(orbus_interface::UnavPIDConfig &config, uint32_t level) {

//    motor_pid_t pid;
//    pid.kp = (float) config.Kp;
//    pid.ki = (float) config.Ki;
//    pid.kd = (float) config.Kd;
//    pid.frequency = (uint32_t) config.Frequency;
//    pid.enable = (uint8_t) config.Enable;

//    //The first time we're called, we just want to make sure we have the
//    //original configuration
//    if(!setup_)
//    {
//      last_pid_ = pid;
//      default_pid_ = last_pid_;
//      setup_ = true;
//      return;
//    }

//    if(config.restore_defaults) {
//      pid = default_pid_;
//      //if someone sets restore defaults on the parameter server, prevent looping
//      config.restore_defaults = false;
//    }

//    std::vector<packet_information_t> list_send;
//    if(last_pid_.kp != pid.kp || last_pid_.ki != pid.ki || last_pid_.kd != pid.kd) {
//        //list_send.push_back(mSerial->createDataPacket(mCommand.command_message, HASHMAP_MOTOR, (message_abstract_u*) & pid));
//        last_pid_ = pid;
//        config.Kp = pid.kp;
//        config.Ki = pid.ki;
//        config.Kd = pid.kd;
//        config.Frequency = pid.frequency;
//        config.Enable = pid.enable;
//    }

    /// Send to serial
    //mSerial->addPacketSend(mSerial->createDataPacket(mCommand.command_message, HASHMAP_MOTOR, (message_abstract_u*) & pid));
}
