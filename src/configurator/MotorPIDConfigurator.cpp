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

MotorPIDConfigurator::MotorPIDConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, string path, string name, unsigned int type, unsigned int number)
    : GenericConfigurator(nh, serial, number)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + path + "/pid/" + name;

    ROS_DEBUG_STREAM("Param " << path + "/pid/" + name << " has " << mName << " N:" << number);
    // Set command message
//    mCommand.bitset.motor = number;
    mCommand.bitset.command = type;

    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavPIDConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<orbus_interface::UnavPIDConfig>::CallbackType cb = boost::bind(&MotorPIDConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void MotorPIDConfigurator::initConfigurator()
{
    /// Send configuration to board
    message_abstract_u temp;
    temp.motor.pid = getParam();
    /// Call the function in Generic Reconfigurator
    SendParameterToBoard(temp);

//    else {
//        ROS_WARN_STREAM("Any PID paramerter for: " << key << " . Send request to uNav");
//        // Build a packet
//        packet_information_t frame = CREATE_PACKET_RESPONSE(mCommand.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
//        // Add packet in the frame
//        if(mSerial->addFrame(frame)->sendList())
//        {
//            ROS_WARN_STREAM("Write PARAM from uNav");
//        }
//    }
}

void MotorPIDConfigurator::setParam(motor_pid_t pid) {
    bool enable = (pid.enable > 0 ? true : false);
    ROS_DEBUG_STREAM("Parameter [Kp:" << pid.kp << ", Ki:" << pid.ki << ", Kd:" << pid.kd << ", Freq:" << pid.frequency << "Hz, En:" << enable << "]");
    nh_.setParam(mName + "/Kp", ((double) pid.kp));
    nh_.setParam(mName + "/Ki", (double) pid.ki);
    nh_.setParam(mName + "/Kd", (double) pid.kd);
    nh_.setParam(mName + "/Frequency", (int) pid.frequency);
    nh_.setParam(mName + "/Enable", enable);
}

motor_pid_t MotorPIDConfigurator::getParam() {
    motor_pid_t pid;

    double temp_double;
    int temp_int;
    bool temp;

    nh_.getParam(mName + "/Kp", temp_double);
    pid.kp = static_cast<float>(temp_double);
    nh_.getParam(mName + "/Ki", temp_double);
    pid.ki = static_cast<float>(temp_double);
    nh_.getParam(mName + "/Kd", temp_double);
    pid.kd = static_cast<float>(temp_double);
    nh_.getParam(mName + "/Frequency", temp_int);
    pid.frequency = static_cast<uint32_t>(temp_int);
    nh_.getParam(mName + "/Enable", temp);
    pid.enable = (temp == true ? 1 : 0);

    ROS_DEBUG_STREAM("Read param from "<< mName << " [Kp:" << pid.kp << ", Ki:" << pid.ki << ", Kd:" << pid.kd << ", Freq:" << pid.frequency << "Hz, En:" << (int) pid.enable << "]");
    return pid;
}

void MotorPIDConfigurator::reconfigureCB(orbus_interface::UnavPIDConfig &config, uint32_t level) {

    motor_pid_t pid;
    pid.kp = (float) config.Kp;
    pid.ki = (float) config.Ki;
    pid.kd = (float) config.Kd;
    pid.frequency = (uint32_t) config.Frequency;
    pid.enable = (config.Enable == true ? 1 : 0);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
        ROS_DEBUG_STREAM("First setup " << mName);
        last_pid_ = pid;
        default_pid_ = last_pid_;
        default_config = config;
        setup_ = true;
        return;
    }

    if(config.restore_defaults)
    {
      pid = default_pid_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
      ROS_WARN_STREAM("Read default param from "<< mName << " [Kp:" << pid.kp << ", Ki:" << pid.ki << ", Kd:" << pid.kd << ", Freq:" << pid.frequency << "Hz, En:" << (int) pid.enable << "]");
    }

    ROS_DEBUG_STREAM("Send new param from "<< mName << " [Kp:" << pid.kp << ", Ki:" << pid.ki << ", Kd:" << pid.kd << ", Freq:" << pid.frequency << "Hz, En:" << (int) pid.enable << "]");

    /// Send configuration to board
    message_abstract_u temp;
    temp.motor.pid = pid;
    /// Call the function in Generic Reconfigurator
    SendParameterToBoard(temp);

    // Store last value of PID
    last_pid_ = pid;
}
