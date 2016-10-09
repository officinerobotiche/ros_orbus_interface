#ifndef GENERICCONTROLLER_H
#define GENERICCONTROLLER_H

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <orbus_interface/Service.h>
#include <orbus_interface/BoardTime.h>
#include <orbus_interface/Peripheral.h>

#include "hardware/serial_controller.h"

namespace ORInterface
{

class GenericInterface : public diagnostic_updater::DiagnosticTask
{
public:
    GenericInterface(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, orbus::serial_controller *serial);

    void initializeDiagnostic();

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

protected:
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    ros::NodeHandle private_mNh;
    // Serial controller communication
    orbus::serial_controller *mSerial;
    // Status connection
    bool serial_status;
    // Diagnostic
    diagnostic_updater::Updater diagnostic_updater;
    //Name of device
    string code_date, code_version, code_author, code_board_type, code_board_name;
private:
    /**
     * @brief systemFrame
     * @param option
     * @param type
     * @param command
     * @param message
     */
    void systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

    /**
     * @brief peripheralFrame
     * @param option
     * @param type
     * @param command
     * @param message
     */
    void peripheralFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

    void convertGPIO(int16_t data);

    /**
     * @brief service_Callback Internal service to require information from the board connected
     * @param req
     * @param msg
     * @return
     */
    bool service_Callback(orbus_interface::Service::Request &req, orbus_interface::Service::Response &msg_system);

    // Service board
    ros::ServiceServer srv_board;
    // time execution functions
    ros::Publisher pub_time;
    ros::Publisher pub_peripheral;
    // Message for pubblisher
    orbus_interface::BoardTime msg_system;
    orbus_interface::Peripheral msg_peripheral;

};

}

#endif // GENERICCONTROLLER_H
