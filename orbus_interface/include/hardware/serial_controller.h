#ifndef SERIAL_CONTROLLER_H
#define SERIAL_CONTROLLER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <signal.h>

#include <communication/or_message.h>
#include <communication/or_frame.h>

using namespace std;

namespace orbus
{

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
    bool send(packet_t packet);

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

    // The packet received from serial
    packet_t mReceive;

    unsigned char BufferTx[MAX_BUFF_TX];
};

}

#endif // SERIAL_CONTROLLER_H
