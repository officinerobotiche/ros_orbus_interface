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
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>

#include "hardware/serial_controller.h"

#include "hardware/uNavInterface.h"

#include <boost/chrono.hpp>


//include "configurator/MotorPIDConfigurator.h"

typedef boost::chrono::steady_clock time_source;

using namespace std;
using namespace ORInterface;



/**
* Control loop not realtime safe
*/
void controlLoop(uNavInterface &orb,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  orb.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
//  orb.writeCommandsToHardware(elapsed);
}

/**
* Diagnostics loop for ORB boards, not realtime safe
*/
void diagnosticLoop(uNavInterface &orb)
{
  orb.updateDiagnostics();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "unav_interface");
    ros::NodeHandle nh, private_nh("~");

    //Hardware information
    double control_frequency, diagnostic_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 1.0);
    private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

    string serial_port_string;
    int32_t baud_rate;

    private_nh.param<string>("serial_port", serial_port_string, "/dev/ttyUSB0");
    private_nh.param<int32_t>("serial_rate", baud_rate, 115200);

    ROS_INFO_STREAM("Open Serial " << serial_port_string << ":" << baud_rate);

    orbus::serial_controller orbusSerial(serial_port_string, baud_rate);
    // Run the serial controller
    orbusSerial.start();

    uNavInterface interface(nh, private_nh, &orbusSerial);

    interface.initialize();

    controller_manager::ControllerManager cm(&interface, nh);

    // Setup separate queue and single-threaded spinner to process timer callbacks
    // that interface with uNav hardware.
    // This avoids having to lock around hardware access, but precludes realtime safety
    // in the control loop.
    ros::CallbackQueue unav_queue;
    ros::AsyncSpinner unav_spinner(1, &unav_queue);

    time_source::time_point last_time = time_source::now();
    ros::TimerOptions control_timer(
                ros::Duration(1 / control_frequency),
                boost::bind(controlLoop, boost::ref(interface), boost::ref(cm), boost::ref(last_time)),
                &unav_queue);
    ros::Timer control_loop = nh.createTimer(control_timer);

    ros::TimerOptions diagnostic_timer(
                ros::Duration(1 / diagnostic_frequency),
                boost::bind(diagnosticLoop, boost::ref(interface)),
                &unav_queue);
    ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

    unav_spinner.start();

    std::string name_node = ros::this_node::getName();
    ROS_INFO("Started %s", name_node.c_str());

    // Process remainder of ROS callbacks separately, mainly ControlManager related
    ros::spin();

    return 0;
}
