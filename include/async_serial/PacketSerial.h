/* 
 * File:   PacketSerial.h
 * Author: raffaello
 *
 * Created on 06 November 2013, 19:20
 */

#ifndef PACKETSERIAL_H
#define	PACKETSERIAL_H

#include "AsyncSerial.h"
#include "packet/packet.h"

#define HEADER_SYNC '#'
#define HEADER_ASYNC '@'
#define HEAD_PKG 2

#define ERROR_HEADER -3
#define ERROR_LENGTH -4
#define ERROR_DATA -5
#define ERROR_CKS -6
#define ERROR_CMD -7
#define ERROR_NACK -8
#define ERROR_OPTION -9
#define ERROR_PKG -10
#define ERROR_CREATE_PKG -11

#define ERROR_TIMEOUT_SYNC_PACKET -12
#define ERROR_MAX_ASYNC_CALLBACK -15
/**
 * Used internally (pkgimpl)
 */
class AsyncPacketImpl;

/**
 * Thrown if timeout occurs
 */
class packet_exception : public std::runtime_error {
public:

    packet_exception(const std::string& arg, int number) : runtime_error(arg) {
        this->number = number;
    }
    
    int number;
};

class PacketSerial : public AsyncSerial {
public:
    PacketSerial();

    PacketSerial(const std::string& devname, unsigned int baud_rate,
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

    virtual ~PacketSerial();

    /**
     * Write data asynchronously. Returns immediately.
     * \param data to be sent through the serial device
     */
    void writePacket(packet_t packet, unsigned char header = HEADER_SYNC);

    /**
     * Read some data, blocking
     * \return the receive buffer. It is empty if no data is available
     * \throws boost::system::system_error if any error
     * \throws timeout_exception in case of timeout
     */
    packet_t readPacket(const boost::posix_time::millisec& wait_duration = boost::posix_time::millisec(1000));

    /**
     * To allow derived classes to set a read callback
     */
    template <class T> void setAsyncPacketCallback(void(T::*fp)(const packet_t*), T* obj) {
        setAsyncPacketCallback(boost::bind(fp, obj, _1));
    }

    /**
     * To allow derived classes to set a read callback
     */
    void setAsyncPacketCallback(const
            boost::function<void (const packet_t*) >& callback);

    /**
     * To unregister the read callback in the derived class destructor so it
     * does not get called after the derived class destructor but before the
     * base class destructor
     */
    void clearAsyncPacketCallback();
    
    /**
     * 
     * @param data
     * @param len
     */
    std::map<std::string, int> getMapError();

private:

    /**
     * Read callback, stores data in the buffer
     */
    void readCallback(const char *data, size_t len);

    bool decode_pkgs(unsigned char rxchar);
    bool pkg_header(unsigned char rxchar);
    bool pkg_length(unsigned char rxchar);
    bool pkg_data(unsigned char rxchar);
    unsigned char pkg_checksum(volatile unsigned char* Buffer, int FirstIndx, int LastIndx);

    std::map<std::string, int> map_error;
    
    bool async, data_ready;
    packet_t receive_pkg;
    unsigned short index_data;
    bool (PacketSerial::*pkg_parse) (unsigned char inchar);
    boost::mutex readQueueMutex;
    boost::condition_variable readPacketCond;

    boost::shared_ptr<AsyncPacketImpl> pkgimpl;
};

#endif	/* PACKETSERIAL_H */

