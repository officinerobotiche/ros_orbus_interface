/*
 * File:   unav_hwinterface.cpp
 * Author: Raffaello Bonghi
 *
 * Created on April 26, 2015, 12:00 PM
 */

#include <ros/ros.h>
#include "hardware/orb_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop for Husky, not realtime safe
*/
void controlLoop(orb_hardware::ORBHardware &orb,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  orb.reportLoopDuration(elapsed);
  orb.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  orb.writeCommandsToHardware();
}

/**
* Diagnostics loop for Husky, not realtime safe
*/
void diagnosticLoop(orb_hardware::ORBHardware &orb)
{
  orb.updateDiagnostics();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "unav_interface");
    ros::NodeHandle nh, private_nh("~");

    double control_frequency, diagnostic_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 10.0);
    private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

    // Initialize robot hardware and link to controller manager
    orb_hardware::ORBHardware orb(nh, private_nh, control_frequency);
    controller_manager::ControllerManager cm(&orb, nh);

    // Setup separate queue and single-threaded spinner to process timer callbacks
    // that interface with Husky hardware - libhorizon_legacy not threadsafe. This
    // avoids having to lock around hardware access, but precludes realtime safety
    // in the control loop.
    ros::CallbackQueue orb_queue;
    ros::AsyncSpinner orb_spinner(1, &orb_queue);

    time_source::time_point last_time = time_source::now();
    ros::TimerOptions control_timer(
        ros::Duration(1 / control_frequency),
        boost::bind(controlLoop, boost::ref(orb), boost::ref(cm), boost::ref(last_time)),
        &orb_queue);
    ros::Timer control_loop = nh.createTimer(control_timer);

    ros::TimerOptions diagnostic_timer(
        ros::Duration(1 / diagnostic_frequency),
        boost::bind(diagnosticLoop, boost::ref(orb)),
        &orb_queue);
    ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

    orb_spinner.start();

    std::string name_node = ros::this_node::getName();
    ROS_INFO("Started %s", name_node.c_str());

    // Process remainder of ROS callbacks separately, mainly ControlManager related
    ros::spin();

    return 0;
}
