/* 
 * File:   Serial.cpp
 * Author: raffaello
 * 
 * Created on June 2, 2013, 3:26 PM
 */

#include "Serial.h"
#include <string>
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>
#include <list>
#include <exception>

Serial::Serial() : io_(), serial_(io_), timer(io_), timeout(boost::posix_time::seconds(0)) {
    Serial("/dev/ttyUSB0", 115200);
}

//Serial::Serial(const Serial& orig)
//{
//}

Serial::~Serial() {
    serial_.close();
}

Serial::Serial(std::string port, unsigned int baud_rate) : io_(), serial_(io_), timer(io_), timeout(boost::posix_time::seconds(0)) {
    index_data_ = 0;
    pkg_parse_ = &Serial::pkg_header;

    serial_.open(port);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    //lock_.unlock();
    sending = false;

    //Start thread
    bool_receive_pkg_ = true; //active thread
    sync_packet_ = false; //Boolean for async packets

    //void * memset ( void * ptr, int value, size_t num );

    counter_func = 0;
    memset(buffer_error_, 0, BUFF_SERIAL_ERROR);
    memset(receive_pkg_.buffer, '\0', sizeof (abstract_packet_t));
    memset(copy_receive_pkg_.buffer, '\0', sizeof (abstract_packet_t));

    thr_ = new boost::thread(boost::bind(&Serial::th_decoder, this));
    thr_->detach();
}

/***/

packet_t Serial::sendPacket(packet_t packet) {
    return sendPacket(HEADER_SYNC, packet);
}

packet_t Serial::sendPacket(char header, packet_t packet) {
    boost::unique_lock<boost::mutex> lock(mutex_);
    boost::unique_lock<boost::mutex> lock_sync_(mutex_sync_);
    if (header == HEADER_SYNC) {
        // if a another quick call this function wait last finish
        if (sending) cond.wait(lock);
        sending = true;
        //std::cout << "reserved" << std::endl;
        ros::Time start = ros::Time::now();
        this->send_pkg_ = packet;
        pkg_send(header, this->send_pkg_);
        //Start Communication with serial and wait return packet
        sync_wait_.wait(lock_sync_);
        ros::Time stop = ros::Time::now();
        ros::Duration duration = stop - start;
        copy_receive_pkg_.time = duration.sec * 1000000000 + duration.nsec;
        //unlock other function
        cond.notify_one();
        return this->copy_receive_pkg_;
    } else {
        pkg_send(header, packet);
        packet_t null_packet;
        null_packet.length = 0;
        //unlock other function
        cond.notify_all();
        return null_packet;
    }
}

//void Serial::th_decoder(const boost::function<void (packet_t)>& _callback) {

void Serial::th_decoder() {
    int value = 0;
    char c; //Buffer variable for char receive from serial
    bool new_send = false;

    while (bool_receive_pkg_) {
        try {
            setTimeout(boost::posix_time::millisec(300)); //Set timeout for 1 second
            if (sending & new_send) //receive sync packet
            {
                new_send = false;
                pkg_send(HEADER_SYNC, this->send_pkg_);
                //        std::cout << "send packet" << std::endl;
            }
            readTimeout(&c, 1); //wait to read char from serial, if timeout generate a error
            value = decode_pkgs(c);
            if (value == true) {
                new_send = false;
                memcpy(&copy_receive_pkg_, &receive_pkg_, sizeof (packet_t)); //Copy packet
                if (sync_packet_) {
                    sending = false;
                    sync_wait_.notify_one(); //Advise to correct receive sync packet
                } else {
                    //Call callback for decode async packet
                    for (boost::array < boost::function<void (packet_t)>, 10 > ::const_iterator iter = async_functions.begin(); iter != async_functions.end(); ++iter) {
                        boost::function<void (packet_t) > func = (*iter);
                        if (func != NULL) {
                            func(copy_receive_pkg_);
                        }
                    }
                }
            }//      else if (value == ERROR_HEADER) // Error on header packet
                //      {
                //        //        std::cout << "error header" << std::endl;
                //      }
            else if (value == ERROR_CKS) {
                //        std::cout << "Error cks" << std::endl;
                if (sync_packet_) {
                    new_send = true;
                }
            }
        } catch (std::exception& e) {
            if (sending) {
                std::cout << "Timeout " << e.what() << std::endl; //print error timeout
                new_send = true;
            }
            //      else
            //        std::cout << "ASYNC " << e.what() << std::endl; //print error timeout
        }
    }
}

int* Serial::getBufferArray() {
    return buffer_error_;
}

void Serial::readTimeout(char *data, size_t size) {
    if (readData.size() > 0)//If there is some data from a previous read
    {
        std::istream is(&readData);
        size_t toRead = std::min(readData.size(), size); //How many bytes to read?
        is.read(data, toRead);
        data += toRead;
        size -= toRead;
        if (size == 0) return; //If read data was enough, just return
    }
    setupParameters = ReadSetupParameters(data, size);
    performReadSetup(setupParameters);

    //For this code to work, there should always be a timeout, so the
    //request for no timeout is translated into a very long timeout
    if (timeout != boost::posix_time::seconds(0)) timer.expires_from_now(timeout);
    else timer.expires_from_now(boost::posix_time::hours(100000));
    timer.async_wait(boost::bind(&Serial::timeoutExpired, this, boost::asio::placeholders::error));

    result = resultInProgress;
    bytesTransferred = 0;
    for (;;) {
        io_.run_one();
        switch (result) {
            case resultSuccess:
                timer.cancel();
                return;
            case resultTimeoutExpired:
                serial_.cancel();
                throw (timeout_exception("Timeout expired"));
            case resultError:
                timer.cancel();
                serial_.cancel();
                throw (boost::system::system_error(boost::system::error_code(), "Error while reading"));
                //if resultInProgress remain in the loop
        }
    }
}

void Serial::performReadSetup(const ReadSetupParameters & param) {
    if (param.fixedSize) {
        boost::asio::async_read(serial_, boost::asio::buffer(param.data, param.size), boost::bind(
                &Serial::readCompleted, this, boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    } else {
        boost::asio::async_read_until(serial_, readData, param.delim, boost::bind(
                &Serial::readCompleted, this, boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }
}

void Serial::readCompleted(const boost::system::error_code& error, const size_t bytesTransferred) {
    if (!error) {
        result = resultSuccess;
        this->bytesTransferred = bytesTransferred;
        return;
    }
    //In case a asynchronous operation is cancelled due to a timeout,
    //each OS seems to have its way to react.
#ifdef _WIN32
    if (error.value() == 995) return; //Windows spits out error 995
#elif defined(__APPLE__)
    if (error.value() == 45) {
        //Bug on OS X, it might be necessary to repeat the setup
        //http://osdir.com/ml/lib.boost.asio.user/2008-08/msg00004.html
        performReadSetup(setupParameters);
        return;
    }
#else //Linux
    if (error.value() == 125) return; //Linux outputs error 125
#endif
    result = resultError;
}

void Serial::timeoutExpired(const boost::system::error_code & error) {
    if (!error && result == resultInProgress) result = resultTimeoutExpired;
}

void Serial::setTimeout(const boost::posix_time::time_duration & t) {
    timeout = t;
}

/***/

int Serial::decode_pkgs(unsigned char rxchar) {
    return ((*this).*pkg_parse_)(rxchar);
}

int Serial::pkg_header(unsigned char rxchar) {
    if (rxchar == HEADER_SYNC) {
        sync_packet_ = true;
        pkg_parse_ = &Serial::pkg_length;
        return false;
    } else if (rxchar == HEADER_ASYNC) {
        sync_packet_ = false;
        pkg_parse_ = &Serial::pkg_length;
        return false;
    } else {
        return pkg_error(ERROR_HEADER);
    }
}

int Serial::pkg_length(unsigned char rxchar) {
    if (rxchar > MAX_RX_BUFF) {
        return pkg_error(ERROR_LENGTH);
    } else {
        pkg_parse_ = &Serial::pkg_data;
        receive_pkg_.length = rxchar;
        return false;
    }
}

int Serial::pkg_data(unsigned char rxchar) {
    unsigned char cks_clc;
    if ((index_data_ + 1) == (receive_pkg_.length + 1)) {
        pkg_parse_ = &Serial::pkg_header; //Restart parse serial packet
        if ((cks_clc = pkg_checksum(receive_pkg_.buffer, 0, index_data_)) == rxchar) {
            //checksum data
            index_data_ = 0; //flush index array data buffer
            return true;
        } else {
            //      std::cerr << "CKS ric: " << rxchar << " - " << "calc: " << cks_clc << std::endl;
            return pkg_error(ERROR_CKS);
        }
    } else {
        receive_pkg_.buffer[index_data_] = rxchar;
        index_data_++;
        return false;
    }
}

int Serial::pkg_error(int error) {
    // TODO complete task error
    index_data_ = 0;
    pkg_parse_ = &Serial::pkg_header; //Restart parse serial packet
    buffer_error_[(-error - 1)] += 1;
    //throw error;
    //printf("Error number: %d\n", error);
    return error;
}

unsigned char Serial::pkg_checksum(volatile unsigned char* Buffer, int FirstIndx, int LastIndx) {
    unsigned char ChkSum = 0;
    int ChkCnt;
#ifdef DEBUG_CKS
    printf("ChkSum - int: %d\n", ChkSum);
#endif
    for (ChkCnt = FirstIndx; ChkCnt < LastIndx; ChkCnt++) {
#ifdef DEBUG_CKS
        printf("ChkSum - int: %d, number: %d\n", Buffer[ChkCnt], ChkCnt);
#endif
        ChkSum += Buffer[ChkCnt];
    }
    return ChkSum;
}

/**
 * Write a string to the serial device.
 * \param s string to write
 * \throws boost::system::system_error on failure
 */
bool Serial::pkg_send(char header, packet_t packet) {
    /* on packet:
     * ------- -----------------
     * | CMD | |   DATA         |
     * ------- -----------------
     *    1        1 -> n
     */

    int i;
    unsigned char BufferTx[HEAD_PKG + packet.length + 1];
    BufferTx[0] = header;
    BufferTx[1] = packet.length;

    for (i = 0; i < packet.length; i++) {
        BufferTx[i + HEAD_PKG] = packet.buffer[i];
    }

    BufferTx[packet.length + HEAD_PKG] = pkg_checksum(BufferTx, HEAD_PKG, packet.length + HEAD_PKG);
#ifdef DEBUG_CKS
    std::cout << BufferTx << std::endl;
    printf("ChkSum Send: %d\n", BufferTx[packet.length + HEAD_PKG]);
#endif
    return boost::asio::write(serial_, boost::asio::buffer(BufferTx, packet.length + HEAD_PKG + 1));
}

std::list<information_packet_t> Serial::parsing(ppacket send, packet_t packet) {
    int i;
    std::list<information_packet_t> list;
    for (i = 0; i < packet.length; i++) {
        unsigned char option;
        if (packet.buffer[i] > 1)
            option = CHANGE;
        else
            option = packet.buffer[i + 2];
        switch (option) {
            case CHANGE:
                list.push_back(Serial::decode_pkg(send, packet.buffer[i + 1], packet.buffer, i + 2));
                break;
            case REQUEST:
            case ACK:
            case NACK:
                list.push_back(Serial::addPacket(send, packet.buffer[i + 1], packet.buffer[i + 2], NULL));
                break;
            default:
                break;
        }
        i += packet.buffer[i] + 1;
    }
    return list;
}

information_packet_t Serial::addChangePacket(packet_t* send, char command, unsigned char* Buffer, unsigned int position, unsigned int length, abstract_packet_t * packet) {
    information_packet_t packet_temp;
    packet_temp.command = command;
    packet_temp.option = CHANGE;
    memcpy(packet_temp.packet.buffer, Buffer + (position * sizeof (unsigned char)), length);
    if (send != NULL)
        Serial::addPacket(send, command, ACK, NULL);
    return packet_temp;
}

information_packet_t Serial::addPacket(packet_t* send, unsigned char command, unsigned char option, abstract_packet_t * packet) {
    switch (option) {
        case REQUEST:
        case CHANGE:
            return Serial::addRequestPacket(send, command, packet);
            break;
        case ACK:
        case NACK:
            return Serial::addInformationPacket(send, command, option);
            break;
        default:
            throw ERROR_OPTION;
            //pkg_error(ERROR_OPTION);
            break;
    }
}

information_packet_t Serial::addInformationPacket(packet_t* send, unsigned char command, unsigned char option) {
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

information_packet_t Serial::decode_pkg(packet_t* send, char command, unsigned char* Buffer, unsigned int position) {
    switch (command) {
        case MOTOR_L:
        case MOTOR_R:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_MOTOR, NULL);
            break;
        case PID_CONTROL_L:
        case PID_CONTROL_R:
            //pid_control_t* ptr_pid = &pid;
            return Serial::addChangePacket(send, command, Buffer, position, LNG_PID_CONTROL, NULL);
            break;
        case COORDINATE:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_COORDINATE, NULL);
            break;
        case PARAMETER_SYSTEM:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_PARAMETER_SYSTEM, NULL);
            break;
        case PARAMETER_MOTORS:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_PARAMETER_MOTORS, NULL);
            break;
        case PARAMETER_SENSOR:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_PARAMETER_SENSOR, NULL);
            break;
        case CONSTRAINT:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_CONSTRAINT, NULL);
            break;
        case VELOCITY:
        case VELOCITY_MIS:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_VELOCITY, NULL);
            break;
        case ENABLE:
        case ENABLE_SENSOR:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_ENABLE, NULL);
            break;
        case TIME_PROCESS:
        case PRIORITY_PROCESS:
        case FRQ_PROCESS:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_PROCESS, NULL);
            break;
        case SERVICES:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_SERVICES, NULL);
            break;
        case ERROR_SERIAL:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_ERROR_PKG, NULL);
            break;
        case ENABLE_AUTOSEND:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_AUTOSEND, NULL);
            break;
        case SENSOR:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_SENSOR, NULL);
            break;
        case HUMIDITY:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_HUMIDITY, NULL);
            break;
        case INFRARED:
            return Serial::addChangePacket(send, command, Buffer, position, LNG_INFRARED, NULL);
            break;
        default:
            throw ERROR_PKG;
            //pkg_error(ERROR_PKG);
            break;
    }
}

information_packet_t Serial::addRequestPacket(packet_t* send, unsigned char command, abstract_packet_t * packet) {
    switch (command) {
        case MOTOR_L:
        case MOTOR_R:
            return Serial::buildRequestPacket(send, command, LNG_MOTOR, packet);
            break;
        case PID_CONTROL_L:
        case PID_CONTROL_R:
            return Serial::buildRequestPacket(send, command, LNG_PID_CONTROL, packet);
            break;
        case COORDINATE:
            return Serial::buildRequestPacket(send, command, LNG_COORDINATE, packet);
            break;
        case PARAMETER_SYSTEM:
            return Serial::buildRequestPacket(send, command, LNG_PARAMETER_SYSTEM, packet);
            break;
        case PARAMETER_MOTORS:
            return Serial::buildRequestPacket(send, command, LNG_PARAMETER_MOTORS, packet);
            break;
        case PARAMETER_SENSOR:
            return Serial::buildRequestPacket(send, command, LNG_PARAMETER_SENSOR, packet);
            break;
        case CONSTRAINT:
            return Serial::buildRequestPacket(send, command, LNG_CONSTRAINT, packet);
            break;
        case VELOCITY:
        case VELOCITY_MIS:
            return Serial::buildRequestPacket(send, command, LNG_VELOCITY, packet);
            break;
        case ENABLE:
        case ENABLE_SENSOR:
            return Serial::buildRequestPacket(send, command, LNG_ENABLE, packet);
            break;
        case TIME_PROCESS:
        case PRIORITY_PROCESS:
        case FRQ_PROCESS:
            return Serial::buildRequestPacket(send, command, LNG_PROCESS, packet);
            break;
        case SERVICES:
            return Serial::buildRequestPacket(send, command, LNG_SERVICES, packet);
            break;
        case ERROR_SERIAL:
            return Serial::buildRequestPacket(send, command, LNG_ERROR_PKG, packet);
            break;
        case ENABLE_AUTOSEND:
            return Serial::buildRequestPacket(send, command, LNG_AUTOSEND, packet);
            break;
        case SENSOR:
            return Serial::buildRequestPacket(send, command, LNG_SENSOR, packet);
            break;
        case HUMIDITY:
            return Serial::buildRequestPacket(send, command, LNG_HUMIDITY, packet);
            break;
        case INFRARED:
            return Serial::buildRequestPacket(send, command, LNG_INFRARED, packet);
            break;
        default:
            throw ERROR_CREATE_PKG;
            //pkg_error(ERROR_CREATE_PKG);
            break;
    }
}

information_packet_t Serial::buildRequestPacket(ppacket send, unsigned char command, const unsigned int length, abstract_packet_t * packet) {
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
            return Serial::addInformationPacket(send, command, REQUEST);
        }
    }
}