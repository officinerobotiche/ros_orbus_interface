
#include <string>

#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include "hardware/uNavInterface.h"

namespace ORInterface
{

#define NUM_MOTORS 2
// List of motors
joint_t joint[NUM_MOTORS];

uNavInterface::uNavInterface(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, orbus::serial_controller *serial)
    : GenericInterface(nh, private_nh, serial)
{
    /// Added all callback to receive information about messages
    bool initCallback = mSerial->addCallback(&uNavInterface::allMotorsFrame, this, HASHMAP_MOTOR);

    /// Load URDF from robot_description
    if(nh.hasParam("/robot_description"))
    {
        ROS_INFO_STREAM("/robot_description found!");
        std::string urdf_string;
        nh.getParam("/robot_description", urdf_string);
        urdf = urdf::parseURDF(urdf_string);
    } else
    {
        ROS_WARN_STREAM("/robot_description NOT found!");
    }
    // Initialize Joints
    for(unsigned i=0; i< NUM_MOTORS; i++)
    {
        joint[i].effort = 0;
        joint[i].motor = new Motor(private_mNh, serial, i);
        joint[i].position = 0;
        joint[i].velocity = 0;
        joint[i].velocity_command = 0;
    }
}

bool uNavInterface::updateDiagnostics()
{
    if(serial_status)
    {
        ROS_DEBUG_STREAM("Update diagnostic");
        // Force update all diagnostic parts
        diagnostic_updater.force_update();
    }
    else
    {
        ROS_ERROR("Error connection! Try to connect again ...");
        // Send list of Command
        serial_status = mSerial->sendList();
        if(serial_status)
        {
            ROS_INFO("... connected!");
            return true;
        }
    }
    return serial_status;
}

void uNavInterface::initializeMotors()
{
    for(unsigned i=0; i< NUM_MOTORS; i++)
    {
        // Initialize all components
        joint[i].motor->initializeMotor();
        ROS_DEBUG_STREAM("Motor [" << (int) i << "] Initialized");
    }

    // Send list of Command
    serial_status = mSerial->sendList();
}

void uNavInterface::initializeInterfaces()
{
    // Initialize the diagnostic from the primitive object
    initializeDiagnostic();

    for(unsigned i=0; i < NUM_MOTORS; i++)
    {
        ROS_DEBUG_STREAM("Hardware interface: "<< joint[i].motor->mMotorName);
        hardware_interface::JointStateHandle joint_state_handle(joint[i].motor->mMotorName, &joint[i].position, &joint[i].velocity, &joint[i].effort);

        joint_state_interface.registerHandle(joint_state_handle);

        /// Differential drive interface
        hardware_interface::JointHandle joint_handle(joint_state_handle, &joint[i].velocity_command);
        velocity_joint_interface.registerHandle(joint_handle);
        // Setup limits
        joint[i].motor->setupLimits(joint_handle, urdf);

        // reset position joint
        ROS_DEBUG_STREAM("Reset position motor: " << joint[i].motor->mMotorName);
        joint[i].motor->resetPosition(0);

        //Add motor in diagnostic updater
        diagnostic_updater.add(*joint[i].motor);
        ROS_DEBUG_STREAM("Motor [" << (int) i << "] Registered");
    }

    ROS_INFO_STREAM("Send all Constraint configuration");
    // Send list of Command
    serial_status = mSerial->sendList();

    /// Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
}

bool uNavInterface::updateJointsFromHardware()
{
    ROS_DEBUG_STREAM("Get measure from uNav");
    for(unsigned i=0; i < NUM_MOTORS; i++)
    {
        // Get motor
        joint[i].motor->addRequestMeasure();
    }
    //Send all messages
    serial_status = mSerial->sendList();
    return serial_status;
}

bool uNavInterface::writeCommandsToHardware(ros::Duration period)
{
    ROS_DEBUG_STREAM("Write command to uNav");
    for(unsigned i=0; i < NUM_MOTORS; i++)
    {
        // Write command
        joint[i].motor->writeCommandsToHardware(period, joint[i].velocity_command);
    }
    //Send all messages
    serial_status = mSerial->sendList();
    return serial_status;
}

void uNavInterface::allMotorsFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message)
{

    motor_command_map_t motor;
    motor.command_message = command;
    int number_motor = (int) motor.bitset.motor;
    ROS_DEBUG_STREAM("Frame [Option: " << option << ", HashMap: " << type << ", Nmotor: " << number_motor << ", Command: " << (int) motor.bitset.command << "]");
    if(number_motor < NUM_MOTORS)
    {
        // Update information
        joint[number_motor].motor->motorFrame(option, type, motor.bitset.command, message.motor);
        // Update status joint
        if(motor.bitset.command == MOTOR_MEASURE)
        {
            joint[number_motor].effort = ((double) message.motor.motor.effort) / 1000.0;
            joint[number_motor].position += message.motor.motor.position_delta;
            joint[number_motor].velocity = ((double)message.motor.motor.velocity) / 1000.0;
        }
    }
    else
    {
        ROS_WARN_STREAM("Error enything motor is initialized for Motor: " << number_motor);
    }

}

}
