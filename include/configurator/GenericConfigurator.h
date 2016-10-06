#ifndef GENERICCONFIGURATOR_H
#define GENERICCONFIGURATOR_H

#include <ros/ros.h>

#include <hardware/serial_controller.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

//template <typename T>
class GenericConfigurator
{
public:
    GenericConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, unsigned int number);

    virtual void initConfigurator() { }

    //virtual void setParam(T) { }
protected:

    void SendParameterToBoard(message_abstract_u message);

protected:
    /// Associate name space
    string mName;
    /// Private namespace
    ros::NodeHandle nh_;
    /// Serial port
    orbus::serial_controller* mSerial;
    /// Command map
    motor_command_map_t mCommand;
    /// Setup variable
    bool setup_;
};

#endif // GENERICCONFIGURATOR_H
