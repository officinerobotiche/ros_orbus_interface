/* 
 * File:   ParserPacket.h
 * Author: raffaello
 *
 * Created on 05 November 2013, 19:29
 */

#ifndef PARSERPACKET_H
#define	PARSERPACKET_H

#include "async_serial/PacketSerial.h"

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
    
    void sendAsyncPacket(packet_t packet);
    
    packet_t sendSyncPacket(packet_t packet, const unsigned int repeat=0, const boost::posix_time::millisec& wait_duration=boost::posix_time::millisec(1000));

    std::list<information_packet_t> parsing(packet_t packet, ppacket send=NULL);
    information_packet_t addPacket(packet_t* send, unsigned char command, unsigned char option, abstract_packet_t * packet);
    
private:
    information_packet_t addChangePacket(packet_t* send, char command, unsigned char* Buffer, unsigned int position, unsigned int length, abstract_packet_t * packet);
    information_packet_t addInformationPacket(packet_t* send, unsigned char command, unsigned char option);
    information_packet_t decode_pkg(packet_t* send, char command, unsigned char* Buffer, unsigned int position);
    information_packet_t addRequestPacket(packet_t* send, unsigned char command, abstract_packet_t * packet);
    information_packet_t buildRequestPacket(ppacket send, unsigned char command, const unsigned int length, abstract_packet_t * packet);

    boost::mutex readPacketMutex;
};

#endif	/* PARSERPACKET_H */

