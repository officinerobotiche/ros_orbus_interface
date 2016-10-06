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

MotorEmergencyConfigurator::MotorEmergencyConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, std::string name, unsigned int number)
    : GenericConfigurator(nh, serial, number)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + name + "/emergency";
    // Set command message
    mCommand.bitset.command = MOTOR_EMERGENCY;

    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavEmergencyConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<orbus_interface::UnavEmergencyConfig>::CallbackType cb = boost::bind(&MotorEmergencyConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void MotorEmergencyConfigurator::initConfigurator()
{
    /// Send configuration to board
    message_abstract_u temp;
    temp.motor.emergency= getParam();
    /// Call the function in Generic Reconfigurator
    SendParameterToBoard(temp);
}

void MotorEmergencyConfigurator::setParam(motor_emergency_t emergency) {
    nh_.setParam(mName + "/Slope_time", emergency.slope_time);
    nh_.setParam(mName + "/Bridge_off", emergency.bridge_off);
    nh_.setParam(mName + "/Timeout", emergency.timeout);
}

motor_emergency_t MotorEmergencyConfigurator::getParam() {
    motor_emergency_t emergency;
    int temp_int;
    double temp_double;

    nh_.getParam(mName + "/Slope_time", temp_double);
    emergency.slope_time = (float) temp_double;

    nh_.getParam(mName + "/Bridge_off", temp_double);
    emergency.bridge_off = (float) temp_double;

    nh_.getParam(mName + "/Timeout", temp_int);
    emergency.timeout = (uint16_t) temp_int;

    return emergency;
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

    /// Send configuration to board
    message_abstract_u temp;
    temp.motor.emergency = emergency;
    /// Call the function in Generic Reconfigurator
    SendParameterToBoard(temp);

    // Store last emergency data
    last_emergency_ = emergency;
}
