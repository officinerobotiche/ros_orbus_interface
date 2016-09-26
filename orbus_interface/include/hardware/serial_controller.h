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

static packet_information_t Save(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

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

    /**
     * @brief addCallback
     * @param callback
     * @param type
     */
    void addCallback(const callback_data_packet_t &callback, unsigned char type);

    /**
     *
     */
    template <class T> void addCallback(packet_information_t(T::*fp)(unsigned char, unsigned char, unsigned char, message_abstract_u), T* obj, unsigned char type) {
        addCallback(bind(fp, obj, _1, _2, _3, _4), type);
    }

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

    friend packet_information_t Save(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message)
    {
        //this->mStopping = 1;

        ROS_INFO_STREAM("Friend function");
        return CREATE_PACKET_EMPTY;
    }

    //friend packet_information_t orbus::serial_controller::Save(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

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

    /// List to send messages to serial
    vector<callback_data_packet_t> list_callback;
};

}

#endif // SERIAL_CONTROLLER_H
