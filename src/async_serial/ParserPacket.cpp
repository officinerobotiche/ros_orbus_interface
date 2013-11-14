/* 
 * File:   ParserPacket.cpp
 * Author: raffaello
 * 
 * Created on 05 November 2013, 19:29
 */

#include "async_serial/ParserPacket.h"

using namespace std;
using namespace boost;

ParserPacket::ParserPacket() : PacketSerial() {
    INITIALIZE_HASHMAP_DEFAULT
    INITIALIZE_HASHMAP_MOTION
    INITIALIZE_HASHMAP_NAVIGATION
}

ParserPacket::ParserPacket(const std::string& devname,
        unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
: PacketSerial(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop) {
    INITIALIZE_HASHMAP_DEFAULT
    INITIALIZE_HASHMAP_MOTION
    INITIALIZE_HASHMAP_NAVIGATION
}

void ParserPacket::sendAsyncPacket(packet_t packet) {
    writePacket(packet, HEADER_ASYNC);
}

packet_t ParserPacket::sendSyncPacket(packet_t packet, const unsigned int repeat, const boost::posix_time::millisec& wait_duration) {
    lock_guard<boost::mutex> l(readPacketMutex);
    writePacket(packet);
    for (int i = 0; i <= repeat; ++i) {
        try {
            return readPacket(wait_duration);
        } catch (...) {
            //Repeat message
        }
    }
    ostringstream convert; // stream used for the conversion
    convert << repeat; // insert the textual representation of 'repeat' in the characters in the stream
    throw (packet_exception("Timeout sync packet n: " + convert.str()));
}

std::vector<information_packet_t> ParserPacket::matching(std::vector<information_packet_t> list_send, const unsigned int repeat, const boost::posix_time::millisec& wait_duration) {
    packet_t pkg_send = encoder(list_send);
    packet_t pkg_return = sendSyncPacket(pkg_send, repeat, wait_duration);
    vector<information_packet_t> list_temp = parsing(pkg_return);
    vector<information_packet_t> list_return;
    for(int i = 0; i < list_temp.size(); ++i) {
        information_packet_t info_send = list_send[i];
        information_packet_t info_receive = list_temp[i];
        if((info_send.command == info_receive.command) && (info_send.type == info_receive.type)) {
            if((info_send.option == REQUEST) && (info_receive.option == DATA)) {
                list_return.push_back(info_receive);
            } else if (info_receive.option != ACK && info_receive.option != DATA ) {
//                string information = "Error packet send: " + info_send.option;
//                information += " receive: " + info_receive.option;
                throw (packet_exception("ERROR"));
            }
        }
    }
    return list_return;
}

vector<information_packet_t> ParserPacket::parsing(packet_t packet_receive) {
    int i;
    vector<information_packet_t> list_data;
    for (i = 0; i < packet_receive.length; ) {
        buffer_packet_u buffer_packet;
        memcpy(&buffer_packet.buffer, &packet_receive.buffer[i], packet_receive.buffer[i]);
        list_data.push_back(buffer_packet.information_packet);
        i += packet_receive.buffer[i];
    }
    return list_data;
}

packet_t ParserPacket::encoder(vector<information_packet_t> list_send) {
    packet_t packet_send;
    packet_send.length = 0;
    for (vector<information_packet_t>::iterator list_iter = list_send.begin(); list_iter != list_send.end(); ++list_iter) {
        buffer_packet_u buffer_packet;
        buffer_packet.information_packet = (*list_iter);
        memcpy(&packet_send.buffer[packet_send.length], &buffer_packet.buffer, buffer_packet.information_packet.length);

        packet_send.length += buffer_packet.information_packet.length;
    }
    return packet_send;
}

packet_t ParserPacket::encoder(information_packet_t *list_send, size_t len) {
    packet_t packet_send;
    packet_send.length = 0;
    for (int i = 0; i < len; ++i) {
        buffer_packet_u buffer_packet;
        buffer_packet.information_packet = list_send[i];

        memcpy(&packet_send.buffer[packet_send.length], &buffer_packet.buffer, buffer_packet.information_packet.length);

        packet_send.length += buffer_packet.information_packet.length;
    }
    return packet_send;
}

packet_t ParserPacket::encoder(information_packet_t send) {
    packet_t packet_send;
    packet_send.length = send.length;
    buffer_packet_u buffer_packet;
    buffer_packet.information_packet = send;
    memcpy(&packet_send.buffer, &buffer_packet.buffer, buffer_packet.information_packet.length);
    return packet_send;
}

information_packet_t ParserPacket::createPacket(unsigned char command, unsigned char option, unsigned char type, abstract_packet_t * packet) {
    information_packet_t information;
    information.command = command;
    information.option = option;
    information.type = type;
    if (option == DATA) {
        switch (type) {
            case HASHMAP_DEFAULT:
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_default[command];
                break;
            case HASHMAP_MOTION:
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_motion[command];
                break;
            case HASHMAP_NAVIGATION:
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_navigation[command];
                break;
            default:
                //TODO trohw
                break;
        }
    } else {
        information.length = LNG_HEAD_INFORMATION_PACKET;
    }
    if (packet != NULL) {
        memcpy(&information.packet, packet, sizeof (abstract_packet_t));
    }
    return information;
}

information_packet_t ParserPacket::createDataPacket(unsigned char command, unsigned char type, abstract_packet_t * packet) {
    return createPacket(command, DATA, type, packet);
}

//information_packet_t ParserPacket::addChangePacket(packet_t* send, char command, unsigned char* Buffer, unsigned int position, unsigned int length, abstract_packet_t * packet) {
//    information_packet_t packet_temp;
//    packet_temp.command = command;
//    packet_temp.option = CHANGE;
//    memcpy(packet_temp.packet.buffer, Buffer + (position * sizeof (unsigned char)), length);
//    if (send != NULL)
//        addPacket(send, command, ACK, NULL);
//    return packet_temp;
//}
//
//information_packet_t ParserPacket::addPacket(packet_t* send, unsigned char command, unsigned char option, abstract_packet_t * packet) {
//    switch (option) {
//        case REQUEST:
//        case CHANGE:
//            return addRequestPacket(send, command, packet);
//            break;
//        case ACK:
//        case NACK:
//            return addInformationPacket(send, command, option);
//            break;
//        default:
//            throw ERROR_OPTION;
//            //pkg_error(ERROR_OPTION);
//            break;
//    }
//}
//
//information_packet_t ParserPacket::addInformationPacket(packet_t* send, unsigned char command, unsigned char option) {
//    if (send != NULL) {
//        send->buffer[send->length] = 1;
//        send->buffer[send->length + 1] = command;
//        send->buffer[send->length + 2] = option;
//        send->length += 3;
//        information_packet_t option_pkg;
//        option_pkg.command = command;
//        option_pkg.option = option;
//        return option_pkg;
//    }
//}
//
//information_packet_t ParserPacket::decode_pkg(packet_t* send, char command, unsigned char* Buffer, unsigned int position) {
//    switch (command) {
//        case MOTOR_L:
//        case MOTOR_R:
//            return addChangePacket(send, command, Buffer, position, LNG_MOTOR, NULL);
//            break;
//        case PID_CONTROL_L:
//        case PID_CONTROL_R:
//            return addChangePacket(send, command, Buffer, position, LNG_PID_CONTROL, NULL);
//            break;
//        case COORDINATE:
//            return addChangePacket(send, command, Buffer, position, LNG_COORDINATE, NULL);
//            break;
//        case PARAMETER_SYSTEM:
//            return addChangePacket(send, command, Buffer, position, LNG_PARAMETER_SYSTEM, NULL);
//            break;
//        case PARAMETER_MOTORS:
//            return addChangePacket(send, command, Buffer, position, LNG_PARAMETER_MOTORS, NULL);
//            break;
//        case PARAMETER_SENSOR:
//            return addChangePacket(send, command, Buffer, position, LNG_PARAMETER_SENSOR, NULL);
//            break;
//        case CONSTRAINT:
//            return addChangePacket(send, command, Buffer, position, LNG_CONSTRAINT, NULL);
//            break;
//        case VELOCITY:
//        case VELOCITY_MIS:
//            return addChangePacket(send, command, Buffer, position, LNG_VELOCITY, NULL);
//            break;
//        case ENABLE:
//        case ENABLE_SENSOR:
//            return addChangePacket(send, command, Buffer, position, LNG_ENABLE, NULL);
//            break;
//        case TIME_PROCESS:
//        case PRIORITY_PROCESS:
//        case FRQ_PROCESS:
//            return addChangePacket(send, command, Buffer, position, LNG_PROCESS, NULL);
//            break;
//        case SERVICES:
//            return addChangePacket(send, command, Buffer, position, LNG_SERVICES, NULL);
//            break;
//        case ERROR_SERIAL:
//            return addChangePacket(send, command, Buffer, position, LNG_ERROR_PKG, NULL);
//            break;
//        case ENABLE_AUTOSEND:
//            return addChangePacket(send, command, Buffer, position, LNG_AUTOSEND, NULL);
//            break;
//        case SENSOR:
//            return addChangePacket(send, command, Buffer, position, LNG_SENSOR, NULL);
//            break;
//        case HUMIDITY:
//            return addChangePacket(send, command, Buffer, position, LNG_HUMIDITY, NULL);
//            break;
//        case INFRARED:
//            return addChangePacket(send, command, Buffer, position, LNG_INFRARED, NULL);
//            break;
//        default:
//            throw ERROR_PKG;
//            //pkg_error(ERROR_PKG);
//            break;
//    }
//}
//
//information_packet_t ParserPacket::addRequestPacket(packet_t* send, unsigned char command, abstract_packet_t * packet) {
//    switch (command) {
//        case MOTOR_L:
//        case MOTOR_R:
//            return buildRequestPacket(send, command, LNG_MOTOR, packet);
//            break;
//        case PID_CONTROL_L:
//        case PID_CONTROL_R:
//            return buildRequestPacket(send, command, LNG_PID_CONTROL, packet);
//            break;
//        case COORDINATE:
//            return buildRequestPacket(send, command, LNG_COORDINATE, packet);
//            break;
//        case PARAMETER_SYSTEM:
//            return buildRequestPacket(send, command, LNG_PARAMETER_SYSTEM, packet);
//            break;
//        case PARAMETER_MOTORS:
//            return buildRequestPacket(send, command, LNG_PARAMETER_MOTORS, packet);
//            break;
//        case PARAMETER_SENSOR:
//            return buildRequestPacket(send, command, LNG_PARAMETER_SENSOR, packet);
//            break;
//        case CONSTRAINT:
//            return buildRequestPacket(send, command, LNG_CONSTRAINT, packet);
//            break;
//        case VELOCITY:
//        case VELOCITY_MIS:
//            return buildRequestPacket(send, command, LNG_VELOCITY, packet);
//            break;
//        case ENABLE:
//        case ENABLE_SENSOR:
//            return buildRequestPacket(send, command, LNG_ENABLE, packet);
//            break;
//        case TIME_PROCESS:
//        case PRIORITY_PROCESS:
//        case FRQ_PROCESS:
//            return buildRequestPacket(send, command, LNG_PROCESS, packet);
//            break;
//        case SERVICES:
//            return buildRequestPacket(send, command, LNG_SERVICES, packet);
//            break;
//        case ERROR_SERIAL:
//            return buildRequestPacket(send, command, LNG_ERROR_PKG, packet);
//            break;
//        case ENABLE_AUTOSEND:
//            return buildRequestPacket(send, command, LNG_AUTOSEND, packet);
//            break;
//        case SENSOR:
//            return buildRequestPacket(send, command, LNG_SENSOR, packet);
//            break;
//        case HUMIDITY:
//            return buildRequestPacket(send, command, LNG_HUMIDITY, packet);
//            break;
//        case INFRARED:
//            return buildRequestPacket(send, command, LNG_INFRARED, packet);
//            break;
//        default:
//            throw ERROR_CREATE_PKG;
//            //pkg_error(ERROR_CREATE_PKG);
//            break;
//    }
//}
//
//information_packet_t ParserPacket::buildRequestPacket(ppacket send, unsigned char command, const unsigned int length, abstract_packet_t * packet) {
//    if (send != NULL) {
//        if (packet != NULL) {
//            information_packet_t request_packet;
//            send->buffer[send->length] = length;
//            send->buffer[send->length + 1] = command;
//            memcpy(send->buffer + (send->length + 2) * sizeof (unsigned char), packet->buffer, length);
//            send->length += length + 2;
//            request_packet.command = command;
//            request_packet.option = REQUEST;
//            memcpy(&request_packet.packet.buffer, packet->buffer, length);
//            return request_packet;
//        } else {
//            return addInformationPacket(send, command, REQUEST);
//        }
//    }
//}
//
