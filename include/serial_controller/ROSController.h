/* 
 * File:   ROSController.h
 * Author: raffaello
 *
 * Created on 13 November 2013, 10:33
 */

#ifndef ROSCONTROLLER_H
#define	ROSCONTROLLER_H

#include <ros/ros.h>
#include <serial_bridge/Service.h>
#include <serial_bridge/Process.h>
#include <serial_bridge/Update.h>
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
    ROSController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial);

    virtual ~ROSController();

    void loadParameter();
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
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

    void addTimerEvent(const boost::function<void (const ros::TimerEvent&) >& callback);

    template <class T> void addTimerEvent(void(T::*fp)(const ros::TimerEvent&), T* obj) {
        addTimerEvent(boost::bind(fp, obj, _1));
    }
    void clearTimerEvent();
    
    std::string getNameBoard();

    //Initialization object
    std::string name_node_; //Name for topics, params, services
    ros::NodeHandle nh_; //NameSpace for bridge controller
    ParserPacket* serial_; //Serial object to comunicate with PIC device
    std::string name_board, version, name_author, compiled;
private:

    typedef boost::function<void (std::vector<information_packet_t>*) > callback_add_packet_t;
    typedef boost::function<void (const ros::TimerEvent&) > callback_timer_event_t;
    callback_add_packet_t callback_add_packet;
    callback_add_packet_t callback_add_parameter;
    callback_timer_event_t callback_timer_event;
    
    ros::Timer timer_;
    ros::ServiceServer srv_board, srv_process;
    ros::Publisher pub_time_process;

    serial_bridge::Process time_process;
    double step_timer, tm_mill, k_time;
    error_pkg_t error_serial;
    int number_process;
    bool init_number_process;


    std::vector<information_packet_t> updatePacket();
    void timerCallback(const ros::TimerEvent& event);

    float getTimeProcess(float process_time);
    void defaultPacket(const unsigned char& command, const abstract_packet_t* packet);

    information_packet_t encodeNameProcess(int number);
    void requestNameProcess();
    information_packet_t encodeServices(char command, unsigned char* buffer = NULL, size_t len = 0);
    void resetBoard(unsigned int repeat = 3);
    void decodeServices(const char command, const unsigned char* buffer);
    
    process_t get_process(std::string name);

    bool processServiceCallback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&);
    bool service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response &msg);
};

#endif	/* ROSCONTROLLER_H */

