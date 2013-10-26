/* 
 * File:   Serial.h
 * Author: raffaello
 *
 * Created on June 2, 2013, 3:26 PM
 */

#ifndef SERIAL_H
#define	SERIAL_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"
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

/**
 * Thrown if timeout occurs	
 */
class timeout_exception : public std::runtime_error {
public:

    timeout_exception(const std::string& arg) : runtime_error(arg) {
    }
};

class Serial : private boost::noncopyable {
public:
    typedef void(* ActionType) (packet_t packet);

    Serial();
    //    Serial(const Serial& orig);
    virtual ~Serial();
    Serial(std::string port, unsigned int baud_rate);

    packet_t sendPacket(packet_t packet);
    packet_t sendPacket(char header, packet_t packet);

    template <class T> void asyncPacket(void(T::*fp)(packet_t), T* obj) {
        const boost::function<void (packet_t)>& _callback = boost::bind(fp, obj, _1);
        async_functions[counter_func] = _callback;
        counter_func++;
    }

    static information_packet_t addPacket(ppacket send, unsigned char command, unsigned char option, abstract_packet_t* packet);

    static std::list<information_packet_t> parsing(ppacket send, packet_t packet);
    void setTimeout(const boost::posix_time::time_duration& t);

    int* getBufferArray();

private:

    /**
     * Parameters of performReadSetup.
     * Just wrapper class, no encapsulation provided
     */
    class ReadSetupParameters {
    public:

        ReadSetupParameters() : fixedSize(false), delim(""), data(0), size(0) {
        }

        explicit ReadSetupParameters(const std::string& delim) :
        fixedSize(false), delim(delim), data(0), size(0) {
        }

        ReadSetupParameters(char *data, size_t size) : fixedSize(true),
        delim(""), data(data), size(size) {
        }
        //Using default copy constructor, operator=
        bool fixedSize; ///< True if need to read a fixed number of parameters
        std::string delim; ///< String end delimiter (valid if fixedSize=false)
        char *data; ///< Pointer to data array (valid if fixedSize=true)
        size_t size; ///< Array size (valid if fixedSize=true)
    };

    /**
     * Possible outcome of a read. Set by callbacks, read from main code
     */
    enum ReadResult {
        resultInProgress,
        resultSuccess,
        resultError,
        resultTimeoutExpired
    };
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    enum ReadResult result; ///< Used by read with timeout
    boost::asio::deadline_timer timer; ///< Timer for timeout
    boost::posix_time::time_duration timeout; ///< Read/write timeout
    ReadSetupParameters setupParameters; ///< Global because used in the OSX fix
    size_t bytesTransferred; ///< Used by async read callback
    boost::asio::streambuf readData; ///< Holds eventual read but not consumed
    boost::array<boost::function<void (packet_t)>, 10 > async_functions;
    int counter_func;

    mutable boost::mutex mutex_; //for fast call function send
    mutable boost::mutex mutex_sync_; //mutex for sync packet
    boost::condition_variable cond, sync_wait_;
    bool sending;
    int (Serial::*pkg_parse_) (unsigned char inchar);
    packet_t send_pkg_, receive_pkg_, copy_receive_pkg_;
    unsigned int index_data_;
    boost::thread* thr_;
    bool bool_receive_pkg_, sync_packet_;
    int buffer_error_[BUFF_SERIAL_ERROR];

    void readTimeout(char *data, size_t size);
    void readCompleted(const boost::system::error_code& error, const size_t bytesTransferred);
    void performReadSetup(const ReadSetupParameters& param);
    void timeoutExpired(const boost::system::error_code& error);
    //Function to encoding packet
    bool pkg_send(char header, packet_t packet);
    static information_packet_t decode_pkg(ppacket send, char command, unsigned char* Buffer, unsigned int position);
    static information_packet_t addChangePacket(packet_t* send, char command, unsigned char* Buffer, unsigned int position, unsigned int length, abstract_packet_t* packet);
    static information_packet_t addInformationPacket(ppacket send, unsigned char command, unsigned char option);
    static information_packet_t addRequestPacket(ppacket send, unsigned char command, abstract_packet_t* packet);
    static information_packet_t buildRequestPacket(ppacket send, unsigned char command, const unsigned int length, abstract_packet_t* packet);

    //Function for decoding packet
    //    void th_decoder(const boost::function<void (packet_t)>& _callback);
    void th_decoder();
    int decode_pkgs(unsigned char rxchar);
    int pkg_header(unsigned char rxchar);
    int pkg_length(unsigned char rxchar);
    int pkg_data(unsigned char rxchar);
    //For encoding and decoding
    unsigned char pkg_checksum(volatile unsigned char* Buffer, int FirstIndx, int LastIndx);
    int pkg_error(int error);
};

#endif	/* SERIAL_H */

