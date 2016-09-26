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

#include <ros/ros.h>

#include "hardware/serial_controller.h"

#include "hardware/uNavController.h"

#include <thread>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "unav_interface");
    ros::NodeHandle nh, private_nh("~");

    //Hardware information
    double control_frequency, diagnostic_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 10.0);
    private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 10.0);

    string serial_port_string;
    int32_t baud_rate;

    private_nh.param<string>("serial_port", serial_port_string, "/dev/ttyUSB0");
    private_nh.param<int32_t>("serial_rate", baud_rate, 115200);

    orbus::serial_controller orbusSerial(serial_port_string, baud_rate);
    // Run the serial controller
    orbusSerial.start();

    ORController::uNavController controller(nh, &orbusSerial);

    motor_command_map_t command;
    command.bitset.motor = 0;
    command.bitset.command = MOTOR_VEL_PID;

    packet_information_t packet = createPacket(command.command_message, PACKET_REQUEST, HASHMAP_MOTOR, NULL, 0);

    orbusSerial.addFrame(packet)->addFrame(packet)->sendList();

    // Process remainder of ROS callbacks separately, mainly ControlManager related
    //ros::spin();

    return 0;
}
