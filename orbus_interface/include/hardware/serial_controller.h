#ifndef SERIAL_CONTROLLER_H
#define SERIAL_CONTROLLER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <signal.h>

#include <or_bus/or_message.h>
#include <or_bus/or_frame.h>

using namespace std;

namespace orbus
{

/// Read complete callback - Array of callback
typedef function<packet_information_t (unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) > callback_data_packet_t;

class serial_controller
{
public:
    /**
     * @brief serial_controller Open the serial controller
     * @param port set the port
     * @param set the baudrate
     */
    serial_controller(string port, unsigned long baudrate);
    /**
     * @brief start Initialize the serial communcation
     * @return if open the connection return true
     */
    bool start();
    /**
     * @brief stop
     * @return
     */
    bool stop();

    /**
     * @brief addCallback
     * @param callback
     * @param type
     */
    bool addCallback(const callback_data_packet_t &callback, unsigned char type);

    /**
     *
     */
    template <class T> bool addCallback(packet_information_t(T::*fp)(unsigned char, unsigned char, unsigned char, message_abstract_u), T* obj, unsigned char type) {
        return addCallback(bind(fp, obj, _1, _2, _3, _4), type);
    }

    serial_controller *addFrame(packet_information_t packet);

    bool sendList();

protected:
    // >>>>> Ctrl+C handler
    /*! Ctrl+C handler
     */
    static void sighandler(int signo)
    {
        serial_controller::mStopping = (signo == SIGINT);
        ROS_INFO_STREAM("Ctrl+C pressed by user" );
    }
    // <<<<< Ctrl+C handler

    /**
     * @brief sendSerialFrame
     * @param list_send
     * @return
     */
    bool sendSerialFrame(vector<packet_information_t> list_send);
    /**
     * @brief sendSerialPacket
     * @param packet
     * @return
     */
    packet_t sendSerialPacket(packet_t packet);

    /*!
     * \brief run
     * Thread function. Process the serial data continously when
     * the \ref Acquisition is called
     */
    void* run();

private:
    /**
     * @brief writePacket Send a packet from serial
     * In a packet we have more messages. A typical data packet
     * have this structure:
     * -------------------------- ---------------------------- -----------------------
     * | Length | CMD | DATA ... | Length | CMD | INFORMATION |Length | CMD | ... ... |
     * -------------------------- ---------------------------- -----------------------
     *    1        2 -> length    length+1 length+2 length+3   ...
     * It is possible to have different type of messages:
     * * Message with data (D)
     * * Message with state information:
     *      * (R) request data
     *      * (A) ack
     *      * (N) nack
     * We have three parts to elaborate and send a new packet (if required)
     * 1. [SAVING] The first part of this function split packets in a
     * list of messages to compute.
     * 2. [COMPUTE] If message have a data in tail, start compute and return a
     * new ACK or NACK message to append in a new packet. If is a request
     * message (R), the new message have in tail the data required.
     * 3. [SEND] Encoding de messages and transform in a packet to send.
     * @param packet the packet to send
     * @return if well written return true
     */
    bool writePacket(packet_t packet);
    /**
     * @brief readPacket
     * @return if received all data in packet return true
     */
    bool readPacket();

private:
    // Serial port object
    serial::Serial mSerial;
    // Serial port name
    string mSerialPort;
    // Serial port baudrate
    uint32_t mBaudrate;
    // Timeout open serial port
    uint32_t mTimeout;

    static bool mStopping; ///< Used to stop driver using Ctrl+C
    bool mStopped; ///< Used to stop the serial processing
    bool mPaused; ///< Used to pause asynchronous data processing

    // The packet received from serial
    packet_t mReceive;
    // buffer to send in Tx transimssion
    unsigned char BufferTx[MAX_BUFF_TX];

    // Hashmap with all type of message
    map<int, callback_data_packet_t> hashmap;

    // List of all frame to send
    vector<packet_information_t> list_send;
};

}

#endif // SERIAL_CONTROLLER_H
