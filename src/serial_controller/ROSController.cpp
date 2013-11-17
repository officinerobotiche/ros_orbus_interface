/* 
 * File:   ROSController.cpp
 * Author: raffaello
 * 
 * Created on 13 November 2013, 10:33
 */

#include "serial_controller/ROSController.h"

using namespace std;

#define NUMBER_PUB 10

ROSController::ROSController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial)
: nh_(nh), name_node_(name_node), serial_(serial), init_number_process(false) {
    serial_->addCallback(&ROSController::defaultPacket, this);
    //    serial_->addErrorCallback(&ROSController::errorPacket, this);

    //Publisher
    pub_time_process = nh_.advertise<serial_bridge::Process>(name_node + "/process", NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    //Services
    srv_board = nh_.advertiseService(name_node + "/service_serial", &ROSController::service_Callback, this);
    srv_process = nh_.advertiseService(name_node + "/process", &ROSController::processServiceCallback, this);

    //Timer
    timer_ = nh_.createTimer(ros::Duration(1), &ROSController::timerCallback, this, false, false);

    map_error_serial[ERROR_TIMEOUT_SYNC_PACKET_STRING] = 0;
    map_error_serial[ERROR_MAX_ASYNC_CALLBACK_STRING] = 0;

    vector<information_packet_t> list_packet;
    list_packet.push_back(encodeServices(VERSION_CODE));
    list_packet.push_back(encodeServices(AUTHOR_CODE));
    list_packet.push_back(encodeServices(NAME_BOARD));
    list_packet.push_back(encodeServices(DATE_CODE));
    serial->parserSendPacket(list_packet);

    if (!nh.hasParam(name_node + "/info/name_board"))
        nh_.setParam(name_node + "/info/name_board", name_board);
}

ROSController::~ROSController() {
    serial_->clearCallback();
    //    serial_->clearErrorCallback();
}

void ROSController::loadParameter() {
    //Name process
    if (nh_.hasParam(name_node_ + "/process/length")) {
        nh_.getParam(name_node_ + "/process/length", number_process);
        time_process.name.resize(number_process);
        time_process.process.resize(number_process);
        for (int i = 0; i < number_process; ++i) {
            ostringstream convert; // stream used for the conversion
            convert << i; // insert the textual representation of 'repeat' in the characters in the stream
            nh_.getParam(name_node_ + "/process/" + convert.str(), time_process.name[i]);
        }
    } else {
        requestNameProcess();
    }
    vector<information_packet_t> list_packet;
    if (nh_.hasParam(name_node_ + "/time")) {
        ROS_DEBUG("Sync parameter /time: load");
        nh_.getParam(name_node_ + "/time/step", step_timer);
        nh_.getParam(name_node_ + "/time/tm_mill", tm_mill);
        nh_.getParam(name_node_ + "/time/k_time", k_time);
    } else {
        ROS_DEBUG("Sync parameter /time: ROBOT -> ROS");
        list_packet.push_back(serial_->createPacket(PARAMETER_SYSTEM, REQUEST));
    }
    //Parameter frequency
    if (nh_.hasParam(name_node_ + "/frequency")) {
        ROS_DEBUG("Sync parameter /frequency: ROS -> ROBOT");
        ROS_INFO("TODO Sync /frequency");
        //        process_t process = get_process("frequency");
        //        list_packet.push_back(serial_->createDataPacket(FRQ_PROCESS, HASHMAP_DEFAULT, (abstract_packet_t*) & process));
    } else {
        ROS_DEBUG("Sync parameter /frequency: ROBOT -> ROS");
        list_packet.push_back(serial_->createPacket(FRQ_PROCESS, REQUEST));
    }
    //Parameter priority
    if (nh_.hasParam(name_node_ + "/priority")) {
        ROS_DEBUG("Sync parameter /priority: ROS -> ROBOT");
        ROS_INFO("TODO Sync /priority");
        //        process_t process = get_process("priority");
        //        list_packet.push_back(serial_->createDataPacket(PRIORITY_PROCESS, HASHMAP_DEFAULT, (abstract_packet_t*) & process));
    } else {
        ROS_DEBUG("Sync parameter /priority: ROBOT -> ROS");
        list_packet.push_back(serial_->createPacket(PRIORITY_PROCESS, REQUEST));
    }
    //Set timer rate
    double rate = 1;
    if (nh_.hasParam(name_node_ + "/rate_timer")) {
        nh_.getParam(name_node_ + "/rate_timer", rate);
        ROS_DEBUG("Sync parameter /rate_timer: load - %f Hz", rate);
    } else {
        nh_.setParam(name_node_ + "/rate_timer", rate);
        ROS_DEBUG("Sync parameter /rate_timer: set - %f Hz", rate);
    }
    timer_.setPeriod(ros::Duration(1 / rate));
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

void ROSController::addVectorPacketRequest(const boost::function<void (std::vector<information_packet_t>*) >& callback) {
    callback_add_packet = callback;
}

void ROSController::clearVectorPacketRequest() {
    callback_add_packet.clear();
}

void ROSController::addParameterPacketRequest(const boost::function<void (std::vector<information_packet_t>*) >& callback) {
    callback_add_parameter = callback;
}

void ROSController::clearParameterPacketRequest() {
    callback_add_parameter.clear();
}

void ROSController::addAliveOperation(const boost::function<bool (const ros::TimerEvent&, std::vector<information_packet_t>*) >& callback, bool start) {
    callback_alive_event = callback;
    if (start)
        timer_.start();
}

void ROSController::clearAliveOperation() {
    callback_alive_event.clear();
}

void ROSController::addTimerEvent(const boost::function<void (const ros::TimerEvent&) >& callback) {
    callback_timer_event = callback;
}

void ROSController::clearTimerEvent() {
    callback_timer_event.clear();
}

std::string ROSController::getNameBoard() {
    return name_board;
}

std::vector<information_packet_t> ROSController::updatePacket() {
    std::vector<information_packet_t> list_packet;
    if (callback_add_packet)
        callback_add_packet(&list_packet);
    if (pub_time_process.getNumSubscribers() >= 1) {
        list_packet.push_back(serial_->createPacket(TIME_PROCESS, REQUEST));
    }
    return list_packet;
}

bool ROSController::aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_packet) {
    if (callback_alive_event) {
        return callback_alive_event(event, list_packet);
    } else if (list_packet->size() != 0) {
        return true;
    } else return false;
}

void ROSController::timerCallback(const ros::TimerEvent& event) {
    vector<information_packet_t> list_packet = updatePacket();
    double rate = 1;
    nh_.getParam(name_node_ + "/rate_timer", rate);
    timer_.setPeriod(ros::Duration(1 / rate));
    if (aliveOperation(event, &list_packet)) {
//        ROS_INFO("Start streaming");
        try {
            serial_->parserSendPacket(list_packet, 3, boost::posix_time::millisec(200));
            if (callback_timer_event)
                callback_timer_event(event);
        } catch (std::exception& e) {
            ROS_ERROR("%s", e.what());
        }
    } else {
        ROS_INFO("Wait user");
        timer_.stop();
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

void ROSController::errorPacket(const unsigned char& command, const abstract_packet_t* packet) {
    ROS_ERROR("Error on command: %d", command);
}

void ROSController::defaultPacket(const unsigned char& command, const abstract_packet_t* packet) {
    switch (command) {
        case SERVICES:
            decodeServices(packet->services.command, &packet->services.buffer[0]);
            break;
        case TIME_PROCESS:
            time_process.idle = getTimeProcess(packet->process.idle);
            time_process.parse_packet = getTimeProcess(packet->process.parse_packet);
            for (int i = 0; i < number_process; ++i) {
                time_process.process[i] = getTimeProcess(packet->process.process[i]);
            }
            pub_time_process.publish(time_process);
            break;
        case PRIORITY_PROCESS:
            for (int i = 0; i < packet->process.length; i++) {
                nh_.setParam(name_node_ + "/priority/" + time_process.name[i], packet->process.process[i]);
            }
            nh_.setParam(name_node_ + "/priority/parse", packet->process.parse_packet);
            break;
        case FRQ_PROCESS:
            for (int i = 0; i < packet->process.length; i++) {
                nh_.setParam(name_node_ + "/frequency/" + time_process.name[i], packet->process.process[i]);
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
            for (int i = 0; i < BUFF_SERIAL_ERROR; ++i) {
                string name = getNameError(-(i + 1));
                if (name.compare(" ") != 0)
                    map_error_serial[name] = packet->error_pkg.number[i];
            }
            break;
        case NAME_PROCESS:
            if (init_number_process) {
                number_process = packet->process_name.name;
                nh_.setParam(name_node_ + "/process/length", number_process);
                time_process.name.resize(number_process);
                time_process.process.resize(number_process);
                init_number_process = false;
            } else {
                string name(packet->process_name.buffer);
                time_process.name[packet->process_name.name] = name;
                ostringstream convert; // stream used for the conversion
                convert << packet->process_name.name; // insert the textual representation of 'repeat' in the characters in the stream
                nh_.setParam(name_node_ + "/process/" + convert.str(), name);
            }
            break;
    }
}

std::string ROSController::getNameError(int number) {
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

information_packet_t ROSController::encodeNameProcess(int number) {
    process_buffer_t name_process;
    name_process.name = number;
    return serial_->createDataPacket(NAME_PROCESS, HASHMAP_DEFAULT, (abstract_packet_t*) & name_process);
}

void ROSController::requestNameProcess() {
    vector<information_packet_t> list_name;
    init_number_process = true;
    serial_->parserSendPacket(encodeNameProcess(-1), 3, boost::posix_time::millisec(200));
    for (int i = 0; i < number_process; ++i) {
        list_name.push_back(encodeNameProcess(i));
    }
    serial_->parserSendPacket(list_name, 3, boost::posix_time::millisec(200));
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

std::string ROSController::getBoardSerialError() {
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

bool ROSController::service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response & msg) {
    if (req.name.compare("reset") == 0) {
        resetBoard();
        msg.name = "reset";
    } else if (req.name.compare("version") == 0) {
        string information_string = "Name Board: " + name_board + " " + version + "\n" +
                name_author + " - Build in: " + compiled + "\n";
        msg.name = information_string;
    } else if (req.name.compare("serial_info") == 0) {
        msg.name = getBoardSerialError();
    } else {
        msg.name = "help";
    }
    return true;
}

process_t ROSController::get_process(std::string name) {
    process_t process;
    int temp;
    process.idle = 0;
    for (int i = 0; i < number_process; ++i) {
        nh_.getParam(name_node_ + "/" + name + "/" + time_process.name[i], temp);
        process.process[i] = temp;
    }
    if (name.compare("priority") == 0) {
        nh_.getParam(name_node_ + "/" + name + "/parse", temp);
        process.parse_packet = temp;
    } else process.parse_packet = 0;
    return process;
}

bool ROSController::processServiceCallback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&) {
    std::string name = req.name;
    process_t process;
    std::vector<information_packet_t> list_send;
    ROS_INFO("PROCESS UPDATE");
    if ((name.compare("priority") == 0) || (name.compare(all_string) == 0)) {
        process = get_process("priority");
        list_send.push_back(serial_->createDataPacket(PRIORITY_PROCESS, HASHMAP_DEFAULT, (abstract_packet_t*) & process));
    }
    if ((name.compare("frequency") == 0) || (name.compare(all_string) == 0)) {
        process = get_process("frequency");
        list_send.push_back(serial_->createDataPacket(FRQ_PROCESS, HASHMAP_DEFAULT, (abstract_packet_t*) & process));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}