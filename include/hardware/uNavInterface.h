#ifndef UNAVCONTROLLER_H
#define UNAVCONTROLLER_H

#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>

#include <urdf/model.h>

#include "hardware/Motor.h"
#include "hardware/GenericInterface.h"

namespace ORInterface
{

typedef struct joint
{
    Motor *motor;
    // State of the motor
    double position;
    double velocity;
    double effort;
    double velocity_command;
} joint_t;

class uNavInterface : public GenericInterface, public hardware_interface::RobotHW
{
public:
    uNavInterface(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, orbus::serial_controller *serial);

    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    void write(const ros::Time& time, const ros::Duration& period);

    void read(const ros::Time& time, const ros::Duration& period);

    /**
     * @brief initializeInterfaces Initialize all motors.
     * Add all Control Interface availbles and add in diagnostic task
     */
    void initializeInterfaces();
    /**
     * @brief updateDiagnostics
     */
    bool updateDiagnostics();

    /**
     * @brief initialize
     */
    void initialize();

private:

    void allMotorsFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

    /**
    * @brief service_Callback
    * @param req
    * @param msg
    * @return
    */
    bool service_Callback(orbus_interface::Service::Request &req, orbus_interface::Service::Response &msg);

private:
    /// URDF information about robot
    urdf::Model model;

    /// ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    map<string, Motor*> mMotor;
    map<int, string> mMotorName;

    // Service board
    ros::ServiceServer srv_unav;
};

}

#endif // UNAVCONTROLLER_H
