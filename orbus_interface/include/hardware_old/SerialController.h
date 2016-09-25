#ifndef SERIALCONTROLLER_H
#define SERIALCONTROLLER_H

#include <mutex>
#include "serial_parser_packet/ParserPacket.h"

class SerialController : public ParserPacket
{
public:
    SerialController();

    SerialController(const std::string& devname, unsigned int baud_rate,
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

    void sendList();

    // Send messages
    void addPacketSend(std::vector<packet_information_t> list_packet);
    void addPacketSend(packet_information_t packet);

private:
    /// List to send messages to serial
    std::vector<packet_information_t> list_send_;
    std::mutex values_mutex;
};

#endif // SERIALCONTROLLER_H
