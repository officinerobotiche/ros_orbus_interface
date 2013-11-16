/* 
 * File:   PacketSerial.cpp
 * Author: raffaello
 * 
 * Created on 06 November 2013, 19:20
 */

#include "async_serial/PacketSerial.h"

#include <string>
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>

using namespace std;
using namespace boost;

class AsyncPacketImpl {
public:

    AsyncPacketImpl() : count(0) {
    }

    void sendAsyncPacket(const packet_t* packet) {
        for (int i = 0; i < count; ++i) {
            callback_t callback = async_functions[i];
            if (callback) callback(packet);
        }
    }

    void addAsyncCallback(const boost::function<void (const packet_t*) >& callback) {
        if (count == 10)
            throw (packet_exception("Max async callback", ERROR_MAX_ASYNC_CALLBACK));
        else
            async_functions[count++] = callback;
    }

    void clearAllAsyncCallback() {
        for (int i = 0; i < count; ++i) {
            callback_t callback = async_functions[i];
            callback.clear();
        }
        count = 0;
    }

private:
    /// Read complete callback - Array of callback
    typedef boost::function<void (const packet_t*) > callback_t;
    unsigned int count;
    boost::array<callback_t, 10 > async_functions;
};

PacketSerial::PacketSerial() : AsyncSerial(), async(false), data_ready(false), pkgimpl(new AsyncPacketImpl) {
    pkg_parse = &PacketSerial::pkg_header;
    setReadCallback(boost::bind(&PacketSerial::readCallback, this, _1, _2));
}

PacketSerial::PacketSerial(const std::string& devname,
        unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
: AsyncSerial(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop), async(false), data_ready(false), pkgimpl(new AsyncPacketImpl) {
    pkg_parse = &PacketSerial::pkg_header;
    setReadCallback(boost::bind(&PacketSerial::readCallback, this, _1, _2));
}

void PacketSerial::writePacket(packet_t packet, unsigned char header) {
    /* on packet:
     * ------- -----------------
     * | CMD | |   DATA         |
     * ------- -----------------
     *    1        1 -> n
     */

    int i;
    size_t size = HEAD_PKG + packet.length + 1;
    unsigned char BufferTx[size];
    BufferTx[0] = header;
    BufferTx[1] = packet.length;

    for (i = 0; i < packet.length; i++) {
        BufferTx[i + HEAD_PKG] = packet.buffer[i];
    }

    BufferTx[packet.length + HEAD_PKG] = pkg_checksum(BufferTx, HEAD_PKG, packet.length + HEAD_PKG);

    string data(reinterpret_cast<const char*> (BufferTx), size);
    writeString(data);
}

void PacketSerial::readCallback(const char *data, size_t len) {
    //0 - Read Header
    //1 - Read Length if true is correct length packet
    //2 - Read Data if n+1 is checksum return true
    for (int i = 0; i < len; ++i) {
        try {
            if (decode_pkgs(data[i])) {
                if (async) {
                    //Send callback
                    pkgimpl->sendAsyncPacket(&receive_pkg);
                } else {
                    {
                        //Notify sync
                        lock_guard<boost::mutex> l(readQueueMutex);
                        data_ready = true;
                    }
                    readPacketCond.notify_one();
                }
            }
        } catch (packet_exception& e) {
//            map_error[e.what()] = map_error[e.what()] + 1;
            //Restart to find header
            pkg_parse = &PacketSerial::pkg_header;
        }
    }
}

packet_t PacketSerial::readPacket(const boost::posix_time::millisec& wait_duration) {
    unique_lock<boost::mutex> l(readQueueMutex);
    const boost::system_time timeout = boost::get_system_time() + wait_duration;
    while (!data_ready) {
        if (!readPacketCond.timed_wait(l, timeout))
            throw (packet_exception("Timeout sync packet", ERROR_TIMEOUT_SYNC_PACKET));
    }
    data_ready = false;
    return receive_pkg;
}

void PacketSerial::setAsyncPacketCallback(const boost::function<void (const packet_t*) >& callback) {
    pkgimpl->addAsyncCallback(callback);
}

void PacketSerial::clearAsyncPacketCallback() {
    pkgimpl->clearAllAsyncCallback();
}

std::map<std::string, int> PacketSerial::getMapError() {
    return map_error;
}

bool PacketSerial::decode_pkgs(unsigned char rxchar) {
    return ((*this).*pkg_parse)(rxchar);
}

bool PacketSerial::pkg_header(unsigned char rxchar) {
    if (rxchar == HEADER_SYNC) {
        async = false;
        pkg_parse = &PacketSerial::pkg_length;
        return false;
    } else if (rxchar == HEADER_ASYNC) {
        async = true;
        pkg_parse = &PacketSerial::pkg_length;
        return false;
    } else {
        throw (packet_exception("Error Header", ERROR_HEADER));
    }
}

bool PacketSerial::pkg_length(unsigned char rxchar) {
    if (rxchar > MAX_RX_BUFF) {
        throw (packet_exception("Error length", ERROR_LENGTH));
    } else {
        pkg_parse = &PacketSerial::pkg_data;
        index_data = 0;
        receive_pkg.length = rxchar;
        return false;
    }
}

bool PacketSerial::pkg_data(unsigned char rxchar) {
    unsigned char cks_clc;
    if ((index_data + 1) == (receive_pkg.length + 1)) {
        pkg_parse = &PacketSerial::pkg_header; //Restart parse serial packet
        if ((cks_clc = pkg_checksum(receive_pkg.buffer, 0, index_data)) == rxchar) {
            //checksum data
            index_data = 0; //flush index array data buffer
            return true;
        } else {
            //      std::cerr << "CKS ric: " << rxchar << " - " << "calc: " << cks_clc << std::endl;
            throw (packet_exception("Error Checksum", ERROR_CKS));
        }
    } else {
        receive_pkg.buffer[index_data] = rxchar;
        index_data++;
        return false;
    }
}

unsigned char PacketSerial::pkg_checksum(volatile unsigned char* Buffer, int FirstIndx, int LastIndx) {
    unsigned char ChkSum = 0;
    int ChkCnt;
    for (ChkCnt = FirstIndx; ChkCnt < LastIndx; ChkCnt++) {
        ChkSum += Buffer[ChkCnt];
    }
    return ChkSum;
}

PacketSerial::~PacketSerial() {
    clearReadCallback();
}