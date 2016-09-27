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

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <orbus_interface/UnavPIDConfig.h>

#include "hardware/serial_controller.h"

using namespace std;

class MotorPIDConfigurator {
public:
    /**
     * @brief MotorPIDConfigurator Initialize the dynamic reconfigurator
     * @param nh Nodehandle of the system
     * @param serial serial port
     * @param path original path to start to find all rosparam variable
     * @param name name of the PID configuration
     * @param type type of message required to send the information to the unav
     * @param number number of motor
     */
    MotorPIDConfigurator(const ros::NodeHandle& nh, orbus::serial_controller *serial, string path, string name, unsigned int type, unsigned int number);

    void initConfigurator();

    void setParam(motor_pid_t pid);
    /**
     * @brief getParam from ROSPARAM and save in motor_pid_t function
     * @return t return the motor_pid_t function
     */
    motor_pid_t getParam();

private:
    /// Associate name space
    string mName;
    /// Private namespace
    ros::NodeHandle nh_;
    /// Serial port
    orbus::serial_controller* mSerial;
    /// Command map
    motor_command_map_t mCommand;
    /// PID message
    motor_pid_t last_pid_, default_pid_;

    bool setup_;

    /**
     * @brief dsrv_ server where is located the dynamic reconfigurator
     */
    dynamic_reconfigure::Server<orbus_interface::UnavPIDConfig> *dsrv_;
    /**
     * @brief reconfigureCB when the dynamic reconfigurator change some values start this method
     * @param config variable with all configuration from dynamic reconfigurator
     * @param level
     */
    void reconfigureCB(orbus_interface::UnavPIDConfig &config, uint32_t level);
};
