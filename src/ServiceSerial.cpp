/* 
 * File:   ServiceSerial.cpp
 * Author: raffaello
 * 
 * Created on 26 October 2013, 14:43
 */

#include "ServiceSerial.h"

ServiceSerial::ServiceSerial(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial) : nh_(nh) {
    name_node_ = name_node;
    this->serial_ = serial; // Initialize serial port
    //    serial_->asyncPacket(&ServiceSerial::actionAsync, this);

    services_t version, author, name_board, date;
    version.command = VERSION_CODE;
    author.command = AUTHOR_CODE;
    name_board.command = NAME_BOARD;
    date.command = DATE_CODE;

    std::vector<information_packet_t> list_packet;

    list_packet.push_back(serial_->createDataPacket(SERVICES, HASHMAP_DEFAULT, (abstract_packet_t*) & version));
    list_packet.push_back(serial_->createDataPacket(SERVICES, HASHMAP_DEFAULT, (abstract_packet_t*) & author));
    list_packet.push_back(serial_->createDataPacket(SERVICES, HASHMAP_DEFAULT, (abstract_packet_t*) & name_board));
    list_packet.push_back(serial_->createDataPacket(SERVICES, HASHMAP_DEFAULT, (abstract_packet_t*) & date));

    packet_t send_pkg = serial_->encoder(list_packet);

    try {
        packet_t packet = serial_->sendSyncPacket(send_pkg, 3, boost::posix_time::millisec(200));
        std::vector<information_packet_t> list_send = serial_->parsing(packet);
        //        char* buff_vers, buff_auth, buff_auth, buff_date;
        for (std::vector<information_packet_t>::iterator list_iter = list_send.begin(); list_iter != list_send.end(); list_iter++) {
            information_packet_t packet = (*list_iter);
            if (packet.command == SERVICES && packet.type == HASHMAP_DEFAULT)
                switch (packet.packet.services.command) {
                    case VERSION_CODE:
                        this->version.append((char*) packet.packet.services.buffer);
                        break;
                    case AUTHOR_CODE:
                        this->name_author.append((char*) packet.packet.services.buffer);
                        break;
                    case NAME_BOARD:
                        this->name_board.append((char*) packet.packet.services.buffer);
                        break;
                    case DATE_CODE:
                        this->compiled.append((char*) packet.packet.services.buffer, SERVICE_BUFF);
                        break;
                }
//                ROS_INFO("%c - %s", packet.packet.services.command, packet.packet.services.buffer);
        }
    } catch (std::exception& e) {
        ROS_ERROR("%s", e.what());
    }

    if (nh_.hasParam(name_node_ + "/" + time_string)) {
        ROS_INFO("Sync parameter %s: load", time_string.c_str());
        nh_.getParam("/" + name_node + "/" + time_string + "/" + step_timer_string, step_timer_);
        nh_.getParam("/" + name_node + "/" + time_string + "/" + int_tm_mill_string, tm_mill_);
        nh_.getParam("/" + name_node + "/" + time_string + "/" + k_time_string, k_time_);
    } else {
        ROS_INFO("Sync parameter %s: ROBOT -> ROS", time_string.c_str());
        send_pkg = serial_->encoder(serial_->createPacket(PARAMETER_SYSTEM, REQUEST));
        try {
            packet_t packet = serial_->sendSyncPacket(send_pkg, 3, boost::posix_time::millisec(200));
            std::vector<information_packet_t> configuration = serial_->parsing(packet);
            parameter_system_t parameter_system = getServiceSerial(configuration, PARAMETER_SYSTEM, ' ').parameter_system;
            step_timer_ = parameter_system.step_timer;
            tm_mill_ = parameter_system.int_tm_mill;
            k_time_ = 1 / (step_timer_ / tm_mill_);
            nh_.setParam("/" + name_node + "/" + time_string + "/" + step_timer_string, step_timer_);
            nh_.setParam("/" + name_node + "/" + time_string + "/" + int_tm_mill_string, tm_mill_);
            nh_.setParam("/" + name_node + "/" + time_string + "/" + k_time_string, k_time_);
        } catch (std::exception& e) {
            ROS_ERROR("%s", e.what());
        }
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

abstract_packet_t ServiceSerial::getServiceSerial(std::vector<information_packet_t> configuration, unsigned char command, unsigned char service_command) {
    abstract_packet_t packet;
    //TODO control error to receive packet
    for (std::vector<information_packet_t>::iterator list_iter = configuration.begin(); list_iter != configuration.end(); list_iter++) {
        information_packet_t packet = (*list_iter);
        if ((packet.option == DATA) && (packet.command = command)) {
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
    services_t service;
    service.command = RESET;
    for (int i = 0; i < repeat; i++) {
        packet_t send_pkg = serial_->encoder(serial_->createDataPacket(SERVICES, HASHMAP_DEFAULT, (abstract_packet_t*) & service));
        serial_->sendAsyncPacket(send_pkg);
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
    std::stringstream service_str;
    service_str << "Error list:" << std::endl;
    packet_t send_pkg = serial_->encoder(serial_->createPacket(ERROR_SERIAL, REQUEST));
    try {
        packet_t packet = serial_->sendSyncPacket(send_pkg, 3, boost::posix_time::millisec(200));
        std::vector<information_packet_t> configuration = serial_->parsing(packet);
        int16_t* error_serial = getServiceSerial(configuration, ERROR_SERIAL, ' ').error_pkg.number;
        for (int i = 0; i < BUFF_SERIAL_ERROR; i++) {
            //service_str << "Type: -" << (i + 1) << " - PC n: " << serial_->getBufferArray()[i] << " - PIC n: " << error_serial[i] << std::endl;
            service_str << "Type: -" << (i + 1) << " - PIC n: " << error_serial[i] << std::endl;
        }
    } catch (std::exception& e) {
        ROS_ERROR("%s", e.what());
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