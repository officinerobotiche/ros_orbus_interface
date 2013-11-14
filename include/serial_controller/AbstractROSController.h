/* 
 * File:   AbstractROSController.h
 * Author: raffaello
 *
 * Created on 24 October 2013, 12:20
 */

#ifndef ABSTRACTROSCONTROLLER_H
#define	ABSTRACTROSCONTROLLER_H

#include "../async_serial/ParserPacket.h"
#include "ServiceSerial.h"
#include "ros/ros.h"

#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

const std::string command_string = "command";

class AbstractROSController {
public:
    virtual void loadParameter() = 0;
    virtual void quit(int sig) = 0;
private:
    virtual void parser(ros::Duration time, std::vector<information_packet_t> serial_packet) = 0;
};

#endif	/* ABSTRACTROSCONTROLLER_H */

