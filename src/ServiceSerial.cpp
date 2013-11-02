/* 
 * File:   ServiceSerial.cpp
 * Author: raffaello
 * 
 * Created on 26 October 2013, 14:43
 */

#include "ServiceSerial.h"

ServiceSerial::ServiceSerial(std::string name_node, const ros::NodeHandle& nh, Serial* serial) : nh_(nh) {
    name_node_ = name_node;
    this->serial_ = serial; // Initialize serial port
//    serial_->asyncPacket(&ServiceSerial::actionAsync, this);

    packet_t send_pkg;
    send_pkg.length = 0;
    services_t version, author, name_board, date;
    version.command = VERSION_CODE;
    author.command = AUTHOR_CODE;
    name_board.command = NAME_BOARD;
    date.command = DATE_CODE;
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & version);
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & author);
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & name_board);
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & date);
    std::list<information_packet_t> configuration = Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    char* buff_vers = (char*) getServiceSerial(configuration, SERVICES, VERSION_CODE).services.buffer;
    char* buff_auth = (char*) getServiceSerial(configuration, SERVICES, AUTHOR_CODE).services.buffer;
    char* buff_name = (char*) getServiceSerial(configuration, SERVICES, NAME_BOARD).services.buffer;
    char* buff_date = (char*) getServiceSerial(configuration, SERVICES, DATE_CODE).services.buffer;

    this->name_board.append(buff_name);
    this->name_author.append(buff_auth);
    this->version.append(buff_vers);
    this->compiled.append(buff_date, SERVICE_BUFF);


    if (nh_.hasParam(name_node_ + "/" + time_string)) {
        ROS_INFO("Sync parameter %s: load", time_string.c_str());
        nh_.getParam("/" + name_node + "/" + time_string + "/" + step_timer_string, step_timer_);
        nh_.getParam("/" + name_node + "/" + time_string + "/" + int_tm_mill_string, tm_mill_);
        nh_.getParam("/" + name_node + "/" + time_string + "/" + k_time_string, k_time_);
    } else {
        ROS_INFO("Sync parameter %s: ROBOT -> ROS", time_string.c_str());
        send_pkg.length = 0;
        Serial::addPacket(&send_pkg, PARAMETER_SYSTEM, REQUEST, NULL);
        configuration = Serial::parsing(NULL, serial_->sendPacket(send_pkg));
        parameter_system_t parameter_system = getServiceSerial(configuration, PARAMETER_SYSTEM, ' ').parameter_system;
        step_timer_ = parameter_system.step_timer;
        tm_mill_ = parameter_system.int_tm_mill;
        k_time_ = 1 / (step_timer_ / tm_mill_);
        nh_.setParam("/" + name_node + "/" + time_string + "/" + step_timer_string, step_timer_);
        nh_.setParam("/" + name_node + "/" + time_string + "/" + int_tm_mill_string, tm_mill_);
        nh_.setParam("/" + name_node + "/" + time_string + "/" + k_time_string, k_time_);
    }
    //Services
    srv_board_ = nh_.advertiseService("/" + name_node + "/" + service_string, &ServiceSerial::service_Callback, this);
}

ServiceSerial::ServiceSerial(const ServiceSerial& orig) {
}

ServiceSerial::~ServiceSerial() {
}

void ServiceSerial::actionAsync(packet_t packet) {
    ROS_INFO("Service Serial Async");

}

float ServiceSerial::getTimeProcess(float process_time) {
    double k_time;
    nh_.getParam("/" + name_node_ + "/" + time_string + "/" + k_time_string, k_time);
    if (process_time < 0) {
        double step_timer;
        nh_.getParam("/" + name_node_ + "/" + time_string + "/" + step_timer_string, step_timer);
        return k_time * (step_timer + process_time);
    }
    return k_time*process_time;
}

abstract_packet_t ServiceSerial::getServiceSerial(std::list<information_packet_t> configuration, unsigned char command, unsigned char service_command) {
    abstract_packet_t packet;
    //TODO control error to receive packet
    for (std::list<information_packet_t>::iterator list_iter = configuration.begin(); list_iter != configuration.end(); list_iter++) {
        information_packet_t packet = (*list_iter);
        if ((packet.option == CHANGE) && (packet.command = command)) {
            switch (command) {
                case SERVICES:
                    if (packet.packet.services.command == service_command)
                        return packet.packet;
                    break;
                case PARAMETER_SYSTEM:
                    return packet.packet;
                    break;
                default:
                    ROS_ERROR("Error decode packet");
                    return packet.packet;
                    break;
            }
        }
    }
    return packet;
}

void ServiceSerial::resetBoard(unsigned int repeat) {
    packet_t send_pkg;
    send_pkg.length = 0;
    services_t service;
    service.command = RESET;
    for (int i = 0; i < repeat; i++) {
        Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & service);
        Serial::parsing(NULL, serial_->sendPacket(HEADER_ASYNC, send_pkg));
    }
}

std::string ServiceSerial::getNameBoard() {
    return name_board;
}

std::string ServiceSerial::getAuthor() {
    return name_author;
}

std::string ServiceSerial::getCompiled() {
    return compiled;
}

std::string ServiceSerial::getVersion() {
    return version;
}

std::string ServiceSerial::getErrorSerial() {
    packet_t send_pkg;
    send_pkg.length = 0;
    std::stringstream service_str;
    Serial::addPacket(&send_pkg, ERROR_SERIAL, REQUEST, NULL);
    std::list<information_packet_t> configuration = Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    int16_t* error_serial = getServiceSerial(configuration, ERROR_SERIAL, ' ').error_pkg.number;
    service_str << "Error list:" << std::endl;
    for (int i = 0; i < BUFF_SERIAL_ERROR; i++) {
        service_str << "Type: -" << (i + 1) << " - PC n: " << serial_->getBufferArray()[i] << " - PIC n: " << error_serial[i] << std::endl;
    }
    return service_str.str();
}

bool ServiceSerial::service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response & msg) {
    ROS_INFO("service: %s", req.name.c_str());
    if (req.name.compare(reset_string) == 0) {
        resetBoard(3);
        msg.name = "reset";
    } else if (req.name.compare(version_string) == 0) {
        std::string information_string = "Name Board: " + getNameBoard() + " " + getVersion() + "\n" +
                getAuthor() + " - Build in: " + getCompiled() + "\n";
        msg.name = information_string;
    } else if (req.name.compare(error_serial_string) == 0) {
        msg.name = getErrorSerial();
    }
    return true;
}