/*
 * File:   unav_hwinterface.cpp
 * Author: Raffaello Bonghi
 *
 * Created on April 26, 2015, 12:00 PM
 */

#include <ros/ros.h>
#include "hardware/ORBHardware.h"
#include "hardware/UNAVHardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop not realtime safe
*/
void controlLoop(UNAVHardware &orb,
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
  orb.writeCommandsToHardware(elapsed);
}

/**
* Diagnostics loop for ORB boards, not realtime safe
*/
void diagnosticLoop(UNAVHardware &orb)
{
  orb.updateDiagnostics();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "unav_interface");
    ros::NodeHandle nh, private_nh("~");

    //Hardware information
    double control_frequency, diagnostic_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 10.0);
    private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 10.0);

    // Frequency to send information via serial
    // This is the double frequency of the best control_frequency or diagnostic_frequency;
    double serial_freq = 2*std::max(control_frequency, diagnostic_frequency);
    //Serial port configuration
    std::string serial_port_string;
    double baud_rate;
    bool arduino;
    private_nh.param<std::string>("serial_port", serial_port_string, "/dev/ttyUSB0");
    private_nh.param<double>("serial_rate", baud_rate, 115200);
    private_nh.param<bool>("serial_arduino", arduino, false);
    SerialController* serial;

    ROS_INFO_STREAM("Open Serial " << serial_port_string << ":" << baud_rate);
    try {
        serial = new SerialController(serial_port_string.c_str(), baud_rate);
        //If protocol on arduino
        if(arduino) {
            int arduino = 2;
            ROS_INFO_STREAM("Wait to start Arduino ("<< arduino << " sec) ... ");
            sleep(arduino);
            ROS_INFO("Serial Arduino started");
        }

        ROS_INFO("Frequency serial sender: %f", serial_freq);
        UNAVHardware interface(nh, private_nh, serial, serial_freq);
        controller_manager::ControllerManager cm(&interface, nh);

        // Setup separate queue and single-threaded spinner to process timer callbacks
        // that interface with Husky hardware - libhorizon_legacy not threadsafe. This
        // avoids having to lock around hardware access, but precludes realtime safety
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

    } catch (std::exception &e) {
        serial->close();
        ROS_ERROR("%s", e.what());
    }
    return 0;
}
