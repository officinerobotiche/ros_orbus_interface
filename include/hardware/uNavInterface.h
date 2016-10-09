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

    /**
     * @brief initializeMotors
     */
    void initializeMotors();
    /**
     * @brief initializeInterfaces Initialize all motors.
     * Add all Control Interface availbles and add in diagnostic task
     */
    void initializeInterfaces();
    /**
     * @brief updateJointsFromHardware
     */
    bool updateJointsFromHardware();

    /**
     * @brief writeCommandsToHardware
     * @param period
     */
    bool writeCommandsToHardware(ros::Duration period);

    /**
     * @brief updateDiagnostics
     */
    bool updateDiagnostics();

private:



    void allMotorsFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

private:
    /// URDF information about robot
    //boost::shared_ptr<urdf::ModelInterface> urdf;
    std::string urdf_string;

    /// ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    map<string, Motor*> mMotor;
    vector<string> mMotorName;
};

}

#endif // UNAVCONTROLLER_H
