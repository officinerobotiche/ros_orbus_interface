#ifndef ORB_HARDWARE_H
#define ORB_HARDWARE_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "hardware/SerialController.h"
#include "hardware_interface/robot_hw.h"

/**
 * Thrown if timeout occurs
 */
class controller_exception : public std::runtime_error {
public:

    controller_exception(const std::string& arg) : runtime_error(arg) {
    }
};

class ORBHardware : public hardware_interface::RobotHW {
public:
    ORBHardware(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, SerialController* serial, double frequency);

    void updateDiagnostics();

    virtual ~ORBHardware();


    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    std::string getNameBoard();
    std::string getTypeBoard();

    // Parameters functioins
    void loadParameter();
    void addParameterPacketRequest(const boost::function<void (std::vector<packet_information_t>*) >& callback);

    template <class T> void addParameterPacketRequest(void(T::*fp)(std::vector<packet_information_t>*), T* obj) {
        addParameterPacketRequest(boost::bind(fp, obj, _1));
    }
    void clearParameterPacketRequest();

protected:
    //Initialization object
    ros::NodeHandle nh_; //NameSpace for bridge controller
    ros::NodeHandle private_nh_; //Private NameSpace for bridge controller
    SerialController* serial_; //Serial object to comunicate with PIC device
    std::string name_board_, version_, name_author_, compiled_, type_board_;

    ros::Timer timer_;

    /// Initialize the diagnostic messages
    void initializeDiagnostics();
    // Diagnostic
    diagnostic_updater::Updater diagnostic_updater_;
private:
    typedef boost::function<void (std::vector<packet_information_t>*) > callback_add_packet_t;
    callback_add_packet_t callback_add_parameter;

    void errorPacket(const unsigned char& command, const message_abstract_u* packet);
    void defaultPacket(const unsigned char& command, const message_abstract_u* packet);

    //Timer
    void timerCallback(const ros::TimerEvent& event);
};

#endif // ORB_HARDWARE_H
