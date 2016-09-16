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

#ifndef DIAGNOSTICMOTOR_H
#define DIAGNOSTICMOTOR_H

#include <ros/ros.h>
#include <orbus_msgs/MotorStatus.h>

#include <packet/packet.h>

class DiagnosticMotor
{
public:
    DiagnosticMotor(const ros::NodeHandle& nh, std::string path, std::string name);

    void run(motor_diagnostic_t diagnostic);

private:
    ros::NodeHandle nh_; //NameSpace for bridge controller
    std::string name_;

    orbus_msgs::MotorStatus motor_diagnostic_msg_;
    ros::Publisher pub_diagnostic_;

    void connectCallback(const ros::SingleSubscriberPublisher& pub);
};

#endif // DIAGNOSTICMOTOR_H
