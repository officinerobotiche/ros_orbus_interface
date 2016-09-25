#include "hardware/SerialController.h"

using namespace std;
using namespace boost;

SerialController::SerialController() : ParserPacket()
{

}

SerialController::SerialController(const std::string& devname,
        unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
: ParserPacket(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop) {

}

void SerialController::sendList() {
    if(!list_send_.empty()) {
        values_mutex.lock();
        std::vector<packet_information_t> temp_send;
        temp_send.swap(list_send_);
        list_send_.clear();     ///< Clear list of commands
        this->parserSendPacket(temp_send, 3, boost::posix_time::millisec(200));
        values_mutex.unlock();
    }
}

//void SerialController::addPacketSend(std::vector<packet_information_t> list_packet) {
//    values_mutex.lock();
//    list_send_.push_back(list_packet);
//    values_mutex.unlock();
//}

void SerialController::addPacketSend(packet_information_t packet) {
    values_mutex.lock();
    list_send_.push_back(packet);
    values_mutex.unlock();
}
