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

#ifndef PARSERPACKET_H
#define	PARSERPACKET_H

#include "async_serial/PacketSerial.h"


/**
 * Used internally (pkgimpl)
 */
class ParserPacketImpl;

/**
 * Thrown if timeout occurs
 */
class parser_exception : public std::runtime_error {
public:

    parser_exception(const std::string& arg) : runtime_error(arg) {
    }
};

class ParserPacket : public PacketSerial {
public:

    ParserPacket();

    ParserPacket(const std::string& devname, unsigned int baud_rate,
            boost::asio::serial_port_base::parity opt_parity =
            boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none),
            boost::asio::serial_port_base::character_size opt_csize =
            boost::asio::serial_port_base::character_size(8),
            boost::asio::serial_port_base::flow_control opt_flow =
            boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none),
            boost::asio::serial_port_base::stop_bits opt_stop =
            boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
    
    virtual ~ParserPacket();

    void sendAsyncPacket(packet_t packet);

    packet_t sendSyncPacket(packet_t packet, const unsigned int repeat = 0, const boost::posix_time::millisec& wait_duration = boost::posix_time::millisec(1000));
    void parserSendPacket(std::vector<information_packet_t> list_send, const unsigned int repeat = 0, const boost::posix_time::millisec& wait_duration = boost::posix_time::millisec(1000));
    void parserSendPacket(information_packet_t send, const unsigned int repeat = 0, const boost::posix_time::millisec& wait_duration = boost::posix_time::millisec(1000));

    std::vector<information_packet_t> parsing(packet_t packet_receive);
    packet_t encoder(std::vector<information_packet_t> list_send);
    packet_t encoder(information_packet_t *list_send, size_t len);
    packet_t encoder(information_packet_t list_send);

    information_packet_t createPacket(unsigned char command, unsigned char option, unsigned char type = HASHMAP_DEFAULT, abstract_message_u * packet = NULL);
    information_packet_t createDataPacket(unsigned char command, unsigned char type, abstract_message_u * packet);

    void addCallback(const boost::function<void (const unsigned char&, const abstract_message_u*) >& callback, unsigned char type=HASHMAP_DEFAULT);
    void addErrorCallback(const boost::function<void (const unsigned char&, const abstract_message_u*) >& callback);

    template <class T> void addCallback(void(T::*fp)(const unsigned char&, const abstract_message_u*), T* obj, unsigned char type=HASHMAP_DEFAULT) {
        addCallback(boost::bind(fp, obj, _1, _2), type);
    }
    
    template <class T> void addErrorCallback(void(T::*fp)(const unsigned char&, const abstract_message_u*), T* obj) {
        addErrorCallback(boost::bind(fp, obj, _1, _2));
    }

    void clearCallback(unsigned char type=HASHMAP_DEFAULT);
    void clearErrorCallback();

private:

    void actionAsync(const packet_t* packet);

    boost::mutex readPacketMutex;
    boost::shared_ptr<ParserPacketImpl> parser_impl;
};

#endif	/* PARSERPACKET_H */

