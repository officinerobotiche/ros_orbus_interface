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
}

ParserPacket::ParserPacket(const std::string& devname,
        unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
: PacketSerial(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop) {
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

list<information_packet_t> ParserPacket::parsing(packet_t packet, ppacket send) {
    int i;
    list<information_packet_t> list_data;
    for (i = 0; i < packet.length; i++) {
        unsigned char option;
        if (packet.buffer[i] > 1)
            option = CHANGE;
        else
            option = packet.buffer[i + 2];
        switch (option) {
            case CHANGE:
                list_data.push_back(decode_pkg(send, packet.buffer[i + 1], packet.buffer, i + 2));
                break;
            case REQUEST:
            case ACK:
            case NACK:
                list_data.push_back(addPacket(send, packet.buffer[i + 1], packet.buffer[i + 2], NULL));
                break;
            default:
                break;
        }
        i += packet.buffer[i] + 1;
    }
    return list_data;
}

information_packet_t ParserPacket::addChangePacket(packet_t* send, char command, unsigned char* Buffer, unsigned int position, unsigned int length, abstract_packet_t * packet) {
    information_packet_t packet_temp;
    packet_temp.command = command;
    packet_temp.option = CHANGE;
    memcpy(packet_temp.packet.buffer, Buffer + (position * sizeof (unsigned char)), length);
    if (send != NULL)
        addPacket(send, command, ACK, NULL);
    return packet_temp;
}

information_packet_t ParserPacket::addPacket(packet_t* send, unsigned char command, unsigned char option, abstract_packet_t * packet) {
    switch (option) {
        case REQUEST:
        case CHANGE:
            return addRequestPacket(send, command, packet);
            break;
        case ACK:
        case NACK:
            return addInformationPacket(send, command, option);
            break;
        default:
            throw ERROR_OPTION;
            //pkg_error(ERROR_OPTION);
            break;
    }
}

information_packet_t ParserPacket::addInformationPacket(packet_t* send, unsigned char command, unsigned char option) {
    if (send != NULL) {
        send->buffer[send->length] = 1;
        send->buffer[send->length + 1] = command;
        send->buffer[send->length + 2] = option;
        send->length += 3;
        information_packet_t option_pkg;
        option_pkg.command = command;
        option_pkg.option = option;
        return option_pkg;
    }
}

information_packet_t ParserPacket::decode_pkg(packet_t* send, char command, unsigned char* Buffer, unsigned int position) {
    switch (command) {
        case MOTOR_L:
        case MOTOR_R:
            return addChangePacket(send, command, Buffer, position, LNG_MOTOR, NULL);
            break;
        case PID_CONTROL_L:
        case PID_CONTROL_R:
            return addChangePacket(send, command, Buffer, position, LNG_PID_CONTROL, NULL);
            break;
        case COORDINATE:
            return addChangePacket(send, command, Buffer, position, LNG_COORDINATE, NULL);
            break;
        case PARAMETER_SYSTEM:
            return addChangePacket(send, command, Buffer, position, LNG_PARAMETER_SYSTEM, NULL);
            break;
        case PARAMETER_MOTORS:
            return addChangePacket(send, command, Buffer, position, LNG_PARAMETER_MOTORS, NULL);
            break;
        case PARAMETER_SENSOR:
            return addChangePacket(send, command, Buffer, position, LNG_PARAMETER_SENSOR, NULL);
            break;
        case CONSTRAINT:
            return addChangePacket(send, command, Buffer, position, LNG_CONSTRAINT, NULL);
            break;
        case VELOCITY:
        case VELOCITY_MIS:
            return addChangePacket(send, command, Buffer, position, LNG_VELOCITY, NULL);
            break;
        case ENABLE:
        case ENABLE_SENSOR:
            return addChangePacket(send, command, Buffer, position, LNG_ENABLE, NULL);
            break;
        case TIME_PROCESS:
        case PRIORITY_PROCESS:
        case FRQ_PROCESS:
            return addChangePacket(send, command, Buffer, position, LNG_PROCESS, NULL);
            break;
        case SERVICES:
            return addChangePacket(send, command, Buffer, position, LNG_SERVICES, NULL);
            break;
        case ERROR_SERIAL:
            return addChangePacket(send, command, Buffer, position, LNG_ERROR_PKG, NULL);
            break;
        case ENABLE_AUTOSEND:
            return addChangePacket(send, command, Buffer, position, LNG_AUTOSEND, NULL);
            break;
        case SENSOR:
            return addChangePacket(send, command, Buffer, position, LNG_SENSOR, NULL);
            break;
        case HUMIDITY:
            return addChangePacket(send, command, Buffer, position, LNG_HUMIDITY, NULL);
            break;
        case INFRARED:
            return addChangePacket(send, command, Buffer, position, LNG_INFRARED, NULL);
            break;
        default:
            throw ERROR_PKG;
            //pkg_error(ERROR_PKG);
            break;
    }
}

information_packet_t ParserPacket::addRequestPacket(packet_t* send, unsigned char command, abstract_packet_t * packet) {
    switch (command) {
        case MOTOR_L:
        case MOTOR_R:
            return buildRequestPacket(send, command, LNG_MOTOR, packet);
            break;
        case PID_CONTROL_L:
        case PID_CONTROL_R:
            return buildRequestPacket(send, command, LNG_PID_CONTROL, packet);
            break;
        case COORDINATE:
            return buildRequestPacket(send, command, LNG_COORDINATE, packet);
            break;
        case PARAMETER_SYSTEM:
            return buildRequestPacket(send, command, LNG_PARAMETER_SYSTEM, packet);
            break;
        case PARAMETER_MOTORS:
            return buildRequestPacket(send, command, LNG_PARAMETER_MOTORS, packet);
            break;
        case PARAMETER_SENSOR:
            return buildRequestPacket(send, command, LNG_PARAMETER_SENSOR, packet);
            break;
        case CONSTRAINT:
            return buildRequestPacket(send, command, LNG_CONSTRAINT, packet);
            break;
        case VELOCITY:
        case VELOCITY_MIS:
            return buildRequestPacket(send, command, LNG_VELOCITY, packet);
            break;
        case ENABLE:
        case ENABLE_SENSOR:
            return buildRequestPacket(send, command, LNG_ENABLE, packet);
            break;
        case TIME_PROCESS:
        case PRIORITY_PROCESS:
        case FRQ_PROCESS:
            return buildRequestPacket(send, command, LNG_PROCESS, packet);
            break;
        case SERVICES:
            return buildRequestPacket(send, command, LNG_SERVICES, packet);
            break;
        case ERROR_SERIAL:
            return buildRequestPacket(send, command, LNG_ERROR_PKG, packet);
            break;
        case ENABLE_AUTOSEND:
            return buildRequestPacket(send, command, LNG_AUTOSEND, packet);
            break;
        case SENSOR:
            return buildRequestPacket(send, command, LNG_SENSOR, packet);
            break;
        case HUMIDITY:
            return buildRequestPacket(send, command, LNG_HUMIDITY, packet);
            break;
        case INFRARED:
            return buildRequestPacket(send, command, LNG_INFRARED, packet);
            break;
        default:
            throw ERROR_CREATE_PKG;
            //pkg_error(ERROR_CREATE_PKG);
            break;
    }
}

information_packet_t ParserPacket::buildRequestPacket(ppacket send, unsigned char command, const unsigned int length, abstract_packet_t * packet) {
    if (send != NULL) {
        if (packet != NULL) {
            information_packet_t request_packet;
            send->buffer[send->length] = length;
            send->buffer[send->length + 1] = command;
            memcpy(send->buffer + (send->length + 2) * sizeof (unsigned char), packet->buffer, length);
            send->length += length + 2;
            request_packet.command = command;
            request_packet.option = REQUEST;
            memcpy(&request_packet.packet.buffer, packet->buffer, length);
            return request_packet;
        } else {
            return addInformationPacket(send, command, REQUEST);
        }
    }
}

