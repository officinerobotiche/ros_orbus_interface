#include "hardware/serial_controller.h"

namespace orbus
{

bool serial_controller::mStopping = false;

serial_controller::serial_controller(string port, unsigned long baudrate) : mSerialPort(port), mBaudrate(baudrate)
{
    // >>>>> Ctrl+C handling
    struct sigaction sigAct;
    memset( &sigAct, 0, sizeof(sigAct) );
    sigAct.sa_handler = serial_controller::sighandler;
    sigaction(SIGINT, &sigAct, 0);
    // <<<<< Ctrl+C handling

    orb_message_init(&mReceive);           ///< Initialize buffer serial error
    orb_frame_init();                      ///< Initialize hash map packet

    mTimeout = 500;
}

bool serial_controller::start()
{
    try
    {
        mSerial.setPort(mSerialPort);
        mSerial.open();
        mSerial.setBaudrate(mBaudrate);

        serial::Timeout to = serial::Timeout::simpleTimeout(mTimeout);
        mSerial.setTimeout(to);
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << mSerialPort << " - Error: "  << e.what() );
        return false;
    }

    if(mSerial.isOpen()){
        ROS_INFO_STREAM("Serial Port correctly initialized: " << mSerialPort );
    }
    else
    {
        ROS_ERROR_STREAM( "Serial port not opened: " << mSerialPort );
        return false;
    }

    ROS_INFO_STREAM( "Serial port ready" );

    return true;
}

void* serial_controller::run()
{

}

bool serial_controller::sendSerialPacket(packet_t packet)
{
    // Size of the packet
    int dataSize = (LNG_PACKET_HEADER + packet.length + 1);

    ROS_DEBUG_STREAM( "To be written " << dataSize << " bytes" );
    //Build a message to send to serial
    build_pkg(BufferTx, packet);
    // Send the packet on serial
    int written = mSerial.write(BufferTx, dataSize);

    if ( written != dataSize )
    {
        ROS_WARN_STREAM( "Serial write error. Written " << written << " bytes instead of" << dataSize << " bytes.");
        return false;
    }
    return true;
}

}
