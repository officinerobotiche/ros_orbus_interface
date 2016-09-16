/*
 * Copyright (C) 2016 Officine Robotiche
 * Author: Raffaello Bonghi
 * email:  raffaello@rnext.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "hardware/DiagnosticMotor.h"

DiagnosticMotor::DiagnosticMotor(const ros::NodeHandle &nh, std::string path, std::string name) : nh_(nh), name_(name)
{
    pub_diagnostic_ = nh_.advertise<orbus_msgs::MotorStatus>(path + "/diagnostic", 10,
            boost::bind(&DiagnosticMotor::connectCallback, this, _1));
}

void DiagnosticMotor::connectCallback(const ros::SingleSubscriberPublisher& pub) {
//    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
}

void DiagnosticMotor::run(motor_diagnostic_t diagnostic){
    motor_diagnostic_msg_.header.stamp = ros::Time::now();
    motor_diagnostic_msg_.name_motor = name_;
    motor_diagnostic_msg_.current = diagnostic.current;
    motor_diagnostic_msg_.voltage = diagnostic.volt;
    motor_diagnostic_msg_.temperature = diagnostic.temperature;

    pub_diagnostic_.publish(motor_diagnostic_msg_);
}
