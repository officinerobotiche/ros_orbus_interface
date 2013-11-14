/* 
 * File:   ROSController.cpp
 * Author: raffaello
 * 
 * Created on 13 November 2013, 10:33
 */

#include "serial_controller/ROSController.h"

#include <serial_bridge/Process.h>

using namespace std;

#define NUMBER_PUB 10

ROSController::ROSController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial)
: nh_(nh), name_node_(name_node), serial_(serial) {
    serial_->addDefaultCallback(&ROSController::defaultPacket, this);

    //Publisher
    pub_time_process = nh_.advertise<serial_bridge::Process>(name_node + "/process", NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    //Services
    srv_board = nh_.advertiseService(name_node + "/service_serial", &ROSController::service_Callback, this);

    double rate = 1;
    if (nh_.hasParam(name_node_ + "/rate_timer")) {
        nh_.getParam(name_node_ + "/rate_timer", rate);
        ROS_INFO("Sync parameter /rate_timer: load - %f Hz", rate);
    } else {
        nh_.setParam(name_node_ + "/rate_timer", rate);
        ROS_INFO("Sync parameter /rate_timer: set - %f Hz", rate);
    }

    //Timer
    timer_ = nh_.createTimer(ros::Duration(1 / rate), &ROSController::timerCallback, this, false, false);

    vector<information_packet_t> list_packet;
    list_packet.push_back(encodeServices(VERSION_CODE));
    list_packet.push_back(encodeServices(AUTHOR_CODE));
    list_packet.push_back(encodeServices(NAME_BOARD));
    list_packet.push_back(encodeServices(DATE_CODE));
    serial->parserSendPacket(list_packet);
}

ROSController::~ROSController() {
    serial_->clearCallback();
}

void ROSController::loadParameter() {
    vector<information_packet_t> list_packet;
    if (nh_.hasParam(name_node_ + "/time")) {
        ROS_INFO("Sync parameter /time: load");
        nh_.getParam(name_node_ + "/time/step", step_timer);
        nh_.getParam(name_node_ + "/time/tm_mill", tm_mill);
        nh_.getParam(name_node_ + "/time/k_time", k_time);
    } else {
        ROS_INFO("Sync parameter /time: ROBOT -> ROS");
        list_packet.push_back(serial_->createPacket(PARAMETER_SYSTEM, REQUEST));
    }
    //Parameter frequency
    if (nh_.hasParam(name_node_ + "/frequency")) {
        ROS_DEBUG("Sync parameter /frequency: ROS -> ROBOT");
        ROS_INFO("TODO Sync /frequency");
    } else {
        ROS_DEBUG("Sync parameter /frequency: ROBOT -> ROS");
        list_packet.push_back(serial_->createPacket(FRQ_PROCESS, REQUEST));
    }
    //Parameter priority
    if (nh_.hasParam(name_node_ + "/priority")) {
        ROS_DEBUG("Sync parameter /priority: ROS -> ROBOT");
        ROS_INFO("TODO Sync /priority");
    } else {
        ROS_DEBUG("Sync parameter /priority: ROBOT -> ROS");
        list_packet.push_back(serial_->createPacket(PRIORITY_PROCESS, REQUEST));
    }
    try {
        serial_->parserSendPacket(list_packet, 3, boost::posix_time::millisec(200));
        ROS_INFO("Send");
    } catch (...) {
        ROS_ERROR("Error");
    }
}

void ROSController::updatePacket(std::vector<information_packet_t>* list) {
    if (pub_time_process.getNumSubscribers() >= 1) {
        list->push_back(serial_->createPacket(TIME_PROCESS, REQUEST));
    }
}

void ROSController::timerCallback(const ros::TimerEvent& event) {
    vector<information_packet_t> list_packet;
    updatePacket(&list_packet);
    double rate = 1;
    nh_.getParam(name_node_ + "/rate_timer", rate);
    timer_.setPeriod(ros::Duration(1 / rate));
    if (list_packet.size() == 0) {
        ROS_INFO("Wait user");
        timer_.stop();
    } else {
        //ROS_INFO("Start streaming");
        try {
            serial_->parserSendPacket(list_packet, 3, boost::posix_time::millisec(200));
        } catch (std::exception& e) {
            ROS_ERROR("%s", e.what());
        }
    }
}

void ROSController::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
    timer_.start();
}

float ROSController::getTimeProcess(float process_time) {
    double k_time;
    nh_.getParam(name_node_ + "/time/k_time", k_time);
    if (process_time < 0) {
        double step_timer;
        nh_.getParam(name_node_ + "/time/step", step_timer);
        return k_time * (step_timer + process_time);
    }
    return k_time*process_time;
}

void ROSController::defaultPacket(const unsigned char& command, const abstract_packet_t* packet) {
    serial_bridge::Process time_process;
    switch (command) {
        case SERVICES:
            decodeServices(packet->services.command, &packet->services.buffer[0]);
            break;
        case TIME_PROCESS:
            time_process.idle = getTimeProcess(packet->process.idle);
            time_process.parse_packet = getTimeProcess(packet->process.parse_packet);
            time_process.pid_l = getTimeProcess(packet->process.process[0]);
            time_process.pid_r = getTimeProcess(packet->process.process[1]);
            time_process.velocity = getTimeProcess(packet->process.process[2]);
            time_process.dead_reckoning = getTimeProcess(packet->process.process[3]);
            pub_time_process.publish(time_process);
            break;
        case PRIORITY_PROCESS:
            for (int i = 0; i < packet->process.length; i++) {
                //                nh_.setParam(name_node_ + "/priority/" + string_process_motion[i], packet->process.process[i]);
            }
            nh_.setParam(name_node_ + "/priority/parse", packet->process.parse_packet);
            break;
        case FRQ_PROCESS:
            for (int i = 0; i < packet->process.length; i++) {
                //                nh_.setParam(name_node_ + "/frequency/" + string_process_motion[i], packet->process.process[i]);
            }
            break;
        case PARAMETER_SYSTEM:
            step_timer = packet->parameter_system.step_timer;
            tm_mill = packet->parameter_system.int_tm_mill;
            k_time = 1 / (step_timer / tm_mill);
            nh_.setParam(name_node_ + "/time/step", step_timer);
            nh_.setParam(name_node_ + "/time/tm_mill", tm_mill);
            nh_.setParam(name_node_ + "/time/k_time", k_time);
            break;
        case ERROR_SERIAL:
            error_serial = packet->error_pkg;
            break;
    }
}

information_packet_t ROSController::encodeServices(char command, unsigned char* buffer, size_t len) {
    services_t service;
    service.command = command;
    if (buffer != NULL)
        memcpy(service.buffer, buffer, len);
    return serial_->createDataPacket(SERVICES, HASHMAP_DEFAULT, (abstract_packet_t*) & service);
}

void ROSController::resetBoard(unsigned int repeat) {
    for (int i = 0; i < repeat; i++)
        serial_->sendAsyncPacket(serial_->encoder(encodeServices(RESET)));
}

void ROSController::decodeServices(const char command, const unsigned char* buffer) {
    switch (command) {
        case VERSION_CODE:
            this->version.append((char*) buffer);
            break;
        case AUTHOR_CODE:
            this->name_author.append((char*) buffer);
            break;
        case NAME_BOARD:
            this->name_board.append((char*) buffer);
            break;
        case DATE_CODE:
            this->compiled.append((char*) buffer, SERVICE_BUFF);
            break;
    }
}

bool ROSController::service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response & msg) {
    if (req.name.compare("reset") == 0) {
        resetBoard();
        msg.name = "reset";
    } else if (req.name.compare("version") == 0) {
        string information_string = "Name Board: " + name_board + " " + version + "\n" +
                name_author + " - Build in: " + compiled + "\n";
        msg.name = information_string;
    } else if (req.name.compare("serial_info") == 0) {
        stringstream service_str;
        service_str << "Error list:" << endl;
        for (int i = 0; i < BUFF_SERIAL_ERROR; i++) {
            //service_str << "Type: -" << (i + 1) << " - PC n: " << serial_->getBufferArray()[i] << " - PIC n: " << error_serial[i] << std::endl;
            service_str << "Type: -" << (i + 1) << " - PIC n: " << error_serial.number[i] << endl;
        }
        msg.name = service_str.str();
    } else {
        msg.name = "help";
    }
    return true;
}