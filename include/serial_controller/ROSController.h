/*
 * Copyright (C) 2014 Officine Robotiche
 * Author: Raffaello Bonghi
 * email:  raffaello.bonghi@officinerobotiche.it
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

#ifndef ROSCONTROLLER_H
#define	ROSCONTROLLER_H

#include <ros/ros.h>
#include <ros_serial_bridge/Service.h>
#include <ros_serial_bridge/Process.h>
#include <ros_serial_bridge/Update.h>
#include <std_srvs/Empty.h>
#include "../async_serial/ParserPacket.h"

const std::string command_string = "command";
const std::string measure_string = "measure";
const std::string left_string = "Left";
const std::string right_string = "Right";
const std::string all_string = "all";
const std::string tf_string = "tf";

/**
 * Thrown if timeout occurs
 */
class controller_exception : public std::runtime_error {
public:

    controller_exception(const std::string& arg) : runtime_error(arg) {
    }
};

class ROSController {
public:
    ROSController(const ros::NodeHandle& nh, ParserPacket* serial);

    virtual ~ROSController();

    void loadParameter();
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    std::string getNameBoard();
    std::string getTypeBoard();

    void addVectorPacketRequest(const boost::function<void (std::vector<information_packet_t>*) >& callback);

    template <class T> void addVectorPacketRequest(void(T::*fp)(std::vector<information_packet_t>*), T* obj) {
        addVectorPacketRequest(boost::bind(fp, obj, _1));
    }
    void clearVectorPacketRequest();

    void addParameterPacketRequest(const boost::function<void (std::vector<information_packet_t>*) >& callback);

    template <class T> void addParameterPacketRequest(void(T::*fp)(std::vector<information_packet_t>*), T* obj) {
        addParameterPacketRequest(boost::bind(fp, obj, _1));
    }
    void clearParameterPacketRequest();

    void addAliveOperation(const boost::function<bool (const ros::TimerEvent&, std::vector<information_packet_t>*) >& callback, bool start=false);

    template <class T> void addAliveOperation(bool(T::*fp)(const ros::TimerEvent&, std::vector<information_packet_t>*), T* obj, bool start=false) {
        addAliveOperation(boost::bind(fp, obj, _1, _2), start);
    }
    void clearAliveOperation();

    void addTimerEvent(const boost::function<void (const ros::TimerEvent&) >& callback);

    template <class T> void addTimerEvent(void(T::*fp)(const ros::TimerEvent&), T* obj) {
        addTimerEvent(boost::bind(fp, obj, _1));
    }
    void clearTimerEvent();

protected:
    //Initialization object
    ros::NodeHandle nh_; //NameSpace for bridge controller
    ParserPacket* serial_; //Serial object to comunicate with PIC device
    ros::Timer timer_;
    std::string name_board, version, name_author, compiled, type_board;
private:

    typedef boost::function<void (std::vector<information_packet_t>*) > callback_add_packet_t;
    typedef boost::function<void (const ros::TimerEvent&) > callback_timer_event_t;
    typedef boost::function<bool (const ros::TimerEvent&, std::vector<information_packet_t>*) > callback_add_event_t;
    callback_add_packet_t callback_add_packet, callback_add_parameter;
    callback_add_event_t callback_alive_event;
    callback_timer_event_t callback_timer_event;

    ros::Duration old_time_alive, reset_time_alive, alive_callback_time;
    ros::ServiceServer srv_board, srv_process;
    ros::Publisher pub_time_process;

    ros_serial_bridge::Process time_process;
    double step_timer, tm_mill, k_time;
    int number_process;
    bool init_number_process;
    std::map<std::string, int> map_error_serial;

    bool aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_packet);
    std::vector<information_packet_t> updatePacket();
    void timerCallback(const ros::TimerEvent& event);

    float getTimeProcess(float process_time);
    void errorPacket(const unsigned char& command, const abstract_message_u* packet);
    void defaultPacket(const unsigned char& command, const abstract_message_u* packet);

    std::string getNameError(int number);
    std::string getBoardSerialError();
    information_packet_t encodeNameProcess(int number);
    void requestNameProcess();
    information_packet_t encodeServices(char command, unsigned char* buffer = NULL, size_t len = 0);
    void resetBoard(unsigned int repeat = 3);
    void decodeServices(const char command, const unsigned char* buffer);

    process_t get_process(std::string name);

    bool processServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool service_Callback(ros_serial_bridge::Service::Request &req, ros_serial_bridge::Service::Response &msg);
};

#endif	/* ROSCONTROLLER_H */

