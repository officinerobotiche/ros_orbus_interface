/**
*
*  \author     Raffaello Bonghi <raffaello.bonghi@officinerobotiche.it>
*  \copyright  Copyright (c) 2014-2015, Officine Robotiche, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "hardware/ORBHardware.h"

using namespace std;

#define NUMBER_PUB 10

ORBHardware::ORBHardware(const ros::NodeHandle& nh, ParserPacket* serial)
: nh_(nh), serial_(serial), init_number_process(false), name_board("Nothing"), type_board("Nothing") {
    serial_->addCallback(&ORBHardware::defaultPacket, this);
    serial_->addErrorCallback(&ORBHardware::errorPacket, this);

    //Publisher
    pub_time_process = nh_.advertise<ros_serial_bridge::Process>("process", NUMBER_PUB,
            boost::bind(&ORBHardware::connectCallback, this, _1));
    //Services
    srv_board = nh_.advertiseService("service_serial", &ORBHardware::service_Callback, this);
    //srv_process = nh_.advertiseService("process", &ORBHardware::processServiceCallback, this);

    map_error_serial[ERROR_TIMEOUT_SYNC_PACKET_STRING] = 0;
    map_error_serial[ERROR_MAX_ASYNC_CALLBACK_STRING] = 0;

    vector<information_packet_t> list_packet;
    list_packet.push_back(encodeServices(VERSION_CODE));
    list_packet.push_back(encodeServices(AUTHOR_CODE));
    list_packet.push_back(encodeServices(NAME_BOARD));
    list_packet.push_back(encodeServices(DATE_CODE));
    list_packet.push_back(encodeServices(TYPE_BOARD));
    serial->parserSendPacket(list_packet);

    if (!nh.hasParam("info/type_board"))
        if (type_board.compare("Nothing") != 0)
            nh_.setParam("info/type_board", type_board);
}

ORBHardware::~ORBHardware() {
    serial_->clearCallback();
    serial_->clearErrorCallback();
}

void ORBHardware::loadParameter() {
    //Name process
//    if (nh_.hasParam("process/length")) {
//        nh_.getParam("process/length", number_process);
//        time_process.name.resize(number_process);
//        time_process.process.resize(number_process);
//        for (int i = 0; i < number_process; ++i) {
//            ostringstream convert; // stream used for the conversion
//            convert << i; // insert the textual representation of 'repeat' in the characters in the stream
//            nh_.getParam("process/" + convert.str(), time_process.name[i]);
//        }
//    } else {
//        requestNameProcess();
//    }
    vector<information_packet_t> list_packet;
    if (nh_.hasParam("time")) {
        ROS_INFO("Sync parameter time: load");
        nh_.getParam("time/step", step_timer);
        nh_.getParam("time/tm_mill", tm_mill);
        nh_.getParam("time/k_time", k_time);
    } else {
        ROS_INFO("Sync parameter time: ROBOT -> ROS");
        list_packet.push_back(serial_->createPacket(PARAMETER_SYSTEM, REQUEST));
    }
//    //Parameter frequency
//    if (nh_.hasParam("frequency")) {
//        ROS_INFO("Sync parameter frequency: ROS -> ROBOT");
//        ROS_INFO("TODO Sync frequency");
//        //        process_t process = get_process("frequency");
//        //        list_packet.push_back(serial_->createDataPacket(FRQ_PROCESS, HASHMAP_DEFAULT, (abstract_packet_t*) & process));
//    } else {
//        ROS_INFO("Sync parameter frequency: ROBOT -> ROS");
//        list_packet.push_back(serial_->createPacket(FRQ_PROCESS, REQUEST));
//    }
//    //Parameter priority
//    if (nh_.hasParam("priority")) {
//        ROS_INFO("Sync parameter /priority: ROS -> ROBOT");
//        ROS_INFO("TODO Sync /priority");
//        //        process_t process = get_process("priority");
//        //        list_packet.push_back(serial_->createDataPacket(PRIORITY_PROCESS, HASHMAP_DEFAULT, (abstract_packet_t*) & process));
//    } else {
//        ROS_INFO("Sync parameter /priority: ROBOT -> ROS");
//        list_packet.push_back(serial_->createPacket(PRIORITY_PROCESS, REQUEST));
//    }
    //Add other parameter request
    if (callback_add_parameter)
        callback_add_parameter(&list_packet);
    try {
        ROS_INFO("Sync parameter");
        serial_->parserSendPacket(list_packet, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

/**
* External hook to trigger diagnostic update
*/
void ORBHardware::updateDiagnostics()
{

}

/**
* Update diagnostics with control loop timing information
*/
void ORBHardware::reportLoopDuration(const ros::Duration &duration)
{

}

void ORBHardware::addVectorPacketRequest(const boost::function<void (std::vector<information_packet_t>*) >& callback) {
    callback_add_packet = callback;
}

void ORBHardware::clearVectorPacketRequest() {
    callback_add_packet.clear();
}

void ORBHardware::addParameterPacketRequest(const boost::function<void (std::vector<information_packet_t>*) >& callback) {
    callback_add_parameter = callback;
}

void ORBHardware::clearParameterPacketRequest() {
    callback_add_parameter.clear();
}

void ORBHardware::addTimerEvent(const boost::function<void (const ros::TimerEvent&) >& callback) {
    callback_timer_event = callback;
}

void ORBHardware::clearTimerEvent() {
    callback_timer_event.clear();
}

std::string ORBHardware::getNameBoard() {
    return name_board;
}

std::string ORBHardware::getTypeBoard() {
    return type_board;
}

std::vector<information_packet_t> ORBHardware::updatePacket() {
    std::vector<information_packet_t> list_packet;
    if (callback_add_packet)
        callback_add_packet(&list_packet);
//    if (pub_time_process.getNumSubscribers() >= 1) {
//        list_packet.push_back(serial_->createPacket(TIME_PROCESS, REQUEST));
//    }
    return list_packet;
}

void ORBHardware::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
}

float ORBHardware::getTimeProcess(float process_time) {
    double k_time;
    nh_.getParam("time/k_time", k_time);
    if (process_time < 0) {
        double step_timer;
        nh_.getParam("time/step", step_timer);
        return k_time * (step_timer + process_time);
    }
    return k_time*process_time;
}

void ORBHardware::errorPacket(const unsigned char& command, const abstract_message_u* packet) {
    ROS_ERROR("Error on command: %d", command);
}

void ORBHardware::defaultPacket(const unsigned char& command, const abstract_message_u* packet) {
    switch (command) {
        case SERVICES:
            decodeServices(packet->services.command, &packet->services.buffer[0]);
            break;
//        case TIME_PROCESS:
//            time_process.idle = getTimeProcess(packet->process.idle);
//            time_process.parse_packet = getTimeProcess(packet->process.parse_packet);
//            for (int i = 0; i < number_process; ++i) {
//                time_process.process[i] = getTimeProcess(packet->process.process[i]);
//            }
//            pub_time_process.publish(time_process);
//            break;
//        case PRIORITY_PROCESS:
//            for (int i = 0; i < packet->process.length; i++) {
//                nh_.setParam("priority/" + time_process.name[i], packet->process.process[i]);
//            }
//            nh_.setParam("priority/parse", packet->process.parse_packet);
//            break;
//        case FRQ_PROCESS:
//            for (int i = 0; i < packet->process.length; i++) {
//                nh_.setParam("frequency/" + time_process.name[i], packet->process.process[i]);
//            }
//            break;
//    case NAME_PROCESS:
//        if (init_number_process) {
//            number_process = packet->process_name.name;
//            nh_.setParam("process/length", number_process);
//            time_process.name.resize(number_process);
//            time_process.process.resize(number_process);
//            init_number_process = false;
//        } else {
//            string name(packet->process_name.buffer);
//            time_process.name[packet->process_name.name] = name;
//            ostringstream convert; // stream used for the conversion
//            convert << packet->process_name.name; // insert the textual representation of 'repeat' in the characters in the stream
//            nh_.setParam("process/" + convert.str(), name);
//        }
//        break;
        case PARAMETER_SYSTEM:
            step_timer = packet->parameter_system.step_timer;
            tm_mill = packet->parameter_system.int_tm_mill;
            k_time = 1 / (step_timer / tm_mill);
            nh_.setParam("time/step", step_timer);
            nh_.setParam("time/tm_mill", tm_mill);
            nh_.setParam("time/k_time", k_time);
            break;
        case ERROR_SERIAL:
            for (int i = 0; i < BUFF_SERIAL_ERROR; ++i) {
                string name = getNameError(-(i + 1));
                if (name.compare(" ") != 0)
                    map_error_serial[name] = packet->error_pkg.number[i];
            }
            break;
    }
}

std::string ORBHardware::getNameError(int number) {
    switch (number) {
        case ERROR_FRAMMING:
            return ERROR_FRAMMING_STRING;
            break;
        case ERROR_OVERRUN:
            return ERROR_OVERRUN_STRING;
        case ERROR_HEADER:
            return ERROR_HEADER_STRING;
        case ERROR_LENGTH:
            return ERROR_LENGTH_STRING;
        case ERROR_DATA:
            return ERROR_DATA_STRING;
        case ERROR_CKS:
            return ERROR_CKS_STRING;
        case ERROR_CMD:
            return ERROR_CMD_STRING;
        case ERROR_NACK:
            return ERROR_NACK_STRING;
        case ERROR_OPTION:
            return ERROR_OPTION_STRING;
        case ERROR_PKG:
            return ERROR_PKG_STRING;
        case ERROR_CREATE_PKG:
            return ERROR_CREATE_PKG_STRING;
        default:
            return " ";
            break;
    }
}

//information_packet_t ORBHardware::encodeNameProcess(int number) {
//    process_buffer_t name_process;
//    name_process.name = number;
//    return serial_->createDataPacket(NAME_PROCESS, HASHMAP_DEFAULT, (abstract_message_u*) & name_process);
//}

//void ORBHardware::requestNameProcess() {
//    vector<information_packet_t> list_name;
//    init_number_process = true;
//    serial_->parserSendPacket(encodeNameProcess(-1), 3, boost::posix_time::millisec(200));
//    for (int i = 0; i < number_process; ++i) {
//        list_name.push_back(encodeNameProcess(i));
//    }
//    serial_->parserSendPacket(list_name, 3, boost::posix_time::millisec(200));
//}

information_packet_t ORBHardware::encodeServices(char command, unsigned char* buffer, size_t len) {
    services_t service;
    service.command = command;
    if (buffer != NULL)
        memcpy(service.buffer, buffer, len);
    return serial_->createDataPacket(SERVICES, HASHMAP_DEFAULT, (abstract_message_u*) & service);
}

void ORBHardware::resetBoard(unsigned int repeat) {
    for (int i = 0; i < repeat; i++)
        serial_->sendAsyncPacket(serial_->encoder(encodeServices(RESET)));
}

void ORBHardware::decodeServices(const char command, const unsigned char* buffer) {
    switch (command) {
        case VERSION_CODE:
            this->version.clear();
            this->version.append((char*) buffer);
            break;
        case AUTHOR_CODE:
            this->name_author.clear();
            this->name_author.append((char*) buffer);
            break;
        case NAME_BOARD:
            this->name_board.clear();
            this->name_board.append((char*) buffer);
            break;
        case DATE_CODE:
            this->compiled.clear();
            this->compiled.append((char*) buffer, SERVICE_BUFF);
            break;
        case TYPE_BOARD:
            this->type_board.clear();
            this->type_board.append((char*) buffer);
            break;
    }
}

std::string ORBHardware::getBoardSerialError() {
    try {
        serial_->parserSendPacket(serial_->createPacket(ERROR_SERIAL, REQUEST), 3, boost::posix_time::millisec(200));
    } catch (std::exception& e) {
        ROS_ERROR("%s", e.what());
    }
    stringstream service_str;
    service_str << "Error list:" << endl;
    // map_error_serial
    map<string, int> map_error = serial_->getMapError();
    map<string, int>::iterator map_error_i = map_error_serial.begin();
    for (map<string, int>::iterator ii = map_error.begin(); ii != map_error.end(); ++ii) {
        string name = (*ii).first;
        service_str << "Type: " << (*ii).first;
        service_str << " - PC: " << (*ii).second;
        if (map_error_i != map_error_serial.end()) {
            service_str << " - PIC: " << (*map_error_i).second << endl;
            ++map_error_i;
        } else {
            service_str << endl;
        }
    }
    return service_str.str();
}

bool ORBHardware::service_Callback(ros_serial_bridge::Service::Request &req, ros_serial_bridge::Service::Response & msg) {
    if (req.name.compare("reset") == 0) {
        resetBoard();
        msg.name = "reset";
    } else if (req.name.compare("version") == 0) {
        string information_string = "Name Board: " + name_board + " " + version + "\n" +
                "Type Board: " + type_board + "\n" +
                name_author + " - Build in: " + compiled + "\n";
        msg.name = information_string;
    } else if(req.name.compare("type") == 0) {
        string information_string = "Type board: " + type_board + "\n";
        msg.name = information_string;
    } else if (req.name.compare("serial_info") == 0) {
        msg.name = getBoardSerialError();
    } else {
        msg.name = "HELP, commands: \nversion\ntype\nserial_info\nhelp";
    }
    return true;
}

//process_t ORBHardware::get_process(std::string name) {
//    process_t process;
//    int temp;
//    process.idle = 0;
//    for (int i = 0; i < number_process; ++i) {
//        nh_.getParam(name + "/" + time_process.name[i], temp);
//        process.process[i] = temp;
//    }
//    if (name.compare("priority") == 0) {
//        nh_.getParam(name + "/parse", temp);
//        process.parse_packet = temp;
//    } else process.parse_packet = 0;
//    return process;
//}

//bool ORBHardware::processServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
//    std::string name = req.name;
//    process_t process;
//    std::vector<information_packet_t> list_send;
//    ROS_INFO("PROCESS UPDATE");
//    if ((name.compare("priority") == 0) || (name.compare(all_string) == 0)) {
//        process = get_process("priority");
//        list_send.push_back(serial_->createDataPacket(PRIORITY_PROCESS, HASHMAP_DEFAULT, (abstract_message_u*) & process));
//    }
//    if ((name.compare("frequency") == 0) || (name.compare(all_string) == 0)) {
//        process = get_process("frequency");
//        list_send.push_back(serial_->createDataPacket(FRQ_PROCESS, HASHMAP_DEFAULT, (abstract_message_u*) & process));
//    }
//    try {
//        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
//    } catch (exception &e) {
//        ROS_ERROR("%s", e.what());
//    }
//    return true;
//}
