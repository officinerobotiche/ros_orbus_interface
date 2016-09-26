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

bool serial_controller::stop()
{
    mStopped = true;
}

void serial_controller::addCallback(const callback_data_packet_t &callback, unsigned char type)
{
    //set_frame_reader(type, NULL, serial_controller::Save);
    //set_frame_reader(type, NULL, serial_controller::Save);
    set_frame_reader(type, NULL, orbus::Save);
    list_callback.push_back(callback);
}

void* serial_controller::run()
{

}

bool serial_controller::sendSerialFrame(vector<packet_information_t> list_send)
{
    if(list_send.size())
    {
        // Build a same array with same size of packet to send
        packet_information_t list_data[list_send.size()];
        size_t len = 0;
        // Encode the list of frames
        packet_t packet = encoder(list_send.data(), list_send.size());
        // Send the packet in serial and wait the received data
        packet_t receive = sendSerialPacket(packet);
        // Read all frame and if is true send a packet with all new information
        if(parser(&receive, &list_data[0], &len)) {
            ROS_DEBUG_STREAM("Parsing packet ok!");
            return true;
        }
    }
    return false;
}

packet_t serial_controller::sendSerialPacket(packet_t packet)
{
    if(mSerial.isOpen())
    {
        writePacket(packet);
        if(readPacket())
        {
            return mReceive;
        }
    }
    //return 0;
}

bool serial_controller::writePacket(packet_t packet)
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

bool serial_controller::readPacket()
{
    do {

        if( mStopping )
        {
            stop();
            break;
        }

        if( !mSerial.waitReadable() )
        {
            ROS_ERROR_STREAM( "IMU timeout connecting");
            return false;
        }

        string reply = mSerial.read( mSerial.available() );

        ROS_DEBUG_STREAM( "Received " << reply.size() << " bytes" );

        for (unsigned i=0; i<reply.length(); ++i)
        {
            unsigned char data = reply.at(i);
            if (decode_pkgs(data))
            {
                return true;
            }
        }
    } while(true);
}

//packet_information_t Save(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {


////    //serial_controller* p = reinterpret_cast<serial_controller*>(option, type, command, message);

//////    for(unsigned i=0; i< this->list_callback.size(); ++i)
//////    {
//////        //this->list_callback.at(i);
//////    }
/////

//    ROS_INFO("I'm in save function");
//    return CREATE_PACKET_EMPTY;
//}

}
