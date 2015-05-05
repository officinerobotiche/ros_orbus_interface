/*
 * Copyright (C) 2014 Officine Robotiche
 * Author: Raffaello Bonghi
 * email:  raffaello.bonghi@officinerobotiche.it
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser General Public License
 * (LGPL) version 2.1 which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/lgpl-2.1.html
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 */

#include "async_serial/ParserPacket.h"

using namespace std;
using namespace boost;

#define NUMBER_CALLBACK 3

class ParserPacketImpl {
public:

    ParserPacketImpl() : counter_default(0), counter_error(0) {
    }

    void sendPacket(std::vector<information_packet_t> list_packet) {
        for (vector<information_packet_t>::iterator list_iter = list_packet.begin(); list_iter != list_packet.end(); ++list_iter) {
            information_packet_t packet = (*list_iter);
            switch (packet.option) {
                case NACK:
                    sendDataCallBack(counter_error, packet.command, &packet.packet, data_error_packet_functions);
                    break;
                case DATA:
                    sendToCallback(&packet);
                    break;
            }
        }
    }

    void addCallback(const boost::function<void (const unsigned char&, const abstract_message_u*) >& callback, unsigned char type) {
        if (type == HASHMAP_DEFAULT) {
            addCallBack("Default", &counter_default, data_default_packet_functions, callback);
        } else {
            this->type = type;
            data_other_packet_callback = callback;
        }
    }

    void addErrorCallback(const boost::function<void (const unsigned char&, const abstract_message_u*) >& callback) {
        addCallBack("Error", &counter_error, data_error_packet_functions, callback);
    }

    void clearCallback(unsigned char type) {
        if (type == HASHMAP_DEFAULT)
            clearCallback(&counter_default, data_default_packet_functions);
        else
            data_other_packet_callback.clear();
    }

    void clearErrorCallback() {
        clearCallback(&counter_error, data_error_packet_functions);
    }

private:

    /// Read complete callback - Array of callback
    typedef boost::function<void (const unsigned char&, const abstract_message_u*) > callback_data_packet_t;

    void clearCallback(unsigned int* counter, boost::array<callback_data_packet_t, NUMBER_CALLBACK >& array) {
        for (int i = 0; i < (*counter); ++i) {
            callback_data_packet_t callback = array[i];
            callback.clear();
        }
        counter = 0;
    }

    void addCallBack(string name, unsigned int* counter, boost::array<callback_data_packet_t, NUMBER_CALLBACK >& array, const boost::function<void (const unsigned char&, const abstract_message_u*) >& callback) {
        if (*counter == 10)
            throw (parser_exception("Max callback packet " + name));
        else {
            array[(*counter)++] = callback;
        }
    }

    void sendToCallback(information_packet_t* packet) {
        if (packet->type == HASHMAP_DEFAULT) {
            sendDataCallBack(counter_default, packet->command, &packet->packet, data_default_packet_functions);
        } else if (packet->type == type)
            if (data_other_packet_callback) data_other_packet_callback(packet->command, &packet->packet);
    }

    void sendDataCallBack(unsigned int counter, unsigned char& command, abstract_message_u* packet, boost::array<callback_data_packet_t, NUMBER_CALLBACK > array) {
        for (int i = 0; i < counter; ++i) {
            callback_data_packet_t callback = array[i];
            if (callback)
                callback(command, packet);
        }
    }
    unsigned char type;
    unsigned int counter_default, counter_error;
    callback_data_packet_t data_other_packet_callback;
    boost::array<callback_data_packet_t, NUMBER_CALLBACK > data_default_packet_functions;
    boost::array<callback_data_packet_t, NUMBER_CALLBACK > data_error_packet_functions;
};

ParserPacket::ParserPacket() : PacketSerial(), parser_impl(new ParserPacketImpl) {
    HASHMAP_DEFAULT_INITIALIZE
    INITIALIZE_HASHMAP_MOTION
    HASHMAP_MOTOR_INITIALIZE
    INITIALIZE_HASHMAP_NAVIGATION
    setAsyncPacketCallback(&ParserPacket::actionAsync, this);
}

ParserPacket::ParserPacket(const std::string& devname,
        unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
: PacketSerial(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop), parser_impl(new ParserPacketImpl) {
    HASHMAP_DEFAULT_INITIALIZE
    INITIALIZE_HASHMAP_MOTION
    HASHMAP_MOTOR_INITIALIZE
    INITIALIZE_HASHMAP_NAVIGATION
    setAsyncPacketCallback(&ParserPacket::actionAsync, this);
}

void ParserPacket::sendAsyncPacket(packet_t packet) {
    if (packet.length != 0)
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
    map_error[ERROR_TIMEOUT_SYNC_PACKET_STRING] = map_error[ERROR_TIMEOUT_SYNC_PACKET_STRING] + 1;
    ostringstream convert; // stream used for the conversion
    convert << repeat; // insert the textual representation of 'repeat' in the characters in the stream
    throw (parser_exception("Timeout sync packet n: " + convert.str()));
}

void ParserPacket::actionAsync(const packet_t* packet) {
    parser_impl->sendPacket(parsing(*packet));
}

void ParserPacket::parserSendPacket(vector<information_packet_t> list_send, const unsigned int repeat, const boost::posix_time::millisec& wait_duration) {
    if (!list_send.empty()) {
        packet_t packet = encoder(list_send);
        packet_t receive = sendSyncPacket(packet, repeat, wait_duration);
        parser_impl->sendPacket(parsing(receive));
    }
}

void ParserPacket::parserSendPacket(information_packet_t send, const unsigned int repeat, const boost::posix_time::millisec& wait_duration) {
    if (send.length != 0) {
        packet_t packet = encoder(send);
        packet_t receive = sendSyncPacket(packet, repeat, wait_duration);
        parser_impl->sendPacket(parsing(receive));
    }
}

vector<information_packet_t> ParserPacket::parsing(packet_t packet_receive) {
    int i;
    vector<information_packet_t> list_data;
    for (i = 0; i < packet_receive.length;) {
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

information_packet_t ParserPacket::createPacket(unsigned char command, unsigned char option, unsigned char type, abstract_message_u * packet) {
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
            case HASHMAP_MOTOR:
                motor_command_map_t command_motor;
                command_motor.command_message = command;
                information.length = LNG_HEAD_INFORMATION_PACKET + hashmap_motor[command_motor.bitset.command];
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
        memcpy(&information.packet, packet, sizeof (abstract_message_u));
    }
    return information;
}

information_packet_t ParserPacket::createDataPacket(unsigned char command, unsigned char type, abstract_message_u * packet) {
    return createPacket(command, DATA, type, packet);
}

void ParserPacket::addCallback(const boost::function<void (const unsigned char&, const abstract_message_u*) >& callback, unsigned char type) {
    parser_impl->addCallback(callback, type);
}

void ParserPacket::addErrorCallback(const boost::function<void (const unsigned char&, const abstract_message_u*) >& callback) {
    parser_impl->addErrorCallback(callback);
}

void ParserPacket::clearCallback(unsigned char type) {
    parser_impl->clearCallback(type);
}

void ParserPacket::clearErrorCallback() {
    parser_impl->clearErrorCallback();
}

ParserPacket::~ParserPacket() {
    clearAsyncPacketCallback();
}
