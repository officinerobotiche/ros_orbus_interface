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

#include "async_serial/ParserPacket.h"

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <orbus_interface/UnavConfiguratorParamConfig.h>

class MotorParamConfigurator {
public:
    MotorParamConfigurator(const ros::NodeHandle& nh, std::string name, unsigned int number, ParserPacket* serial);
private:

    /// Associate name space
    std::string name_;
    /// Private namespace
    ros::NodeHandle nh_;
    /// Serial port
    ParserPacket* serial_;
    /// Command map
    motor_command_map_t command_;
    /// Frequency message
    system_task_t last_frequency_, default_frequency_;

    motor_parameter_t last_param_, default_param_;

    bool setup_;

    dynamic_reconfigure::Server<orbus_interface::UnavConfiguratorParamConfig> *dsrv_;
    void reconfigureCB(orbus_interface::UnavConfiguratorParamConfig &config, uint32_t level);
};
