
#include <string>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include "hardware/uNavInterface.h"

namespace ORInterface
{

uNavInterface::uNavInterface(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, orbus::serial_controller *serial)
    : GenericInterface(nh, private_nh, serial)
{
    /// Added all callback to receive information about messages
    bool initCallback = mSerial->addCallback(&uNavInterface::allMotorsFrame, this, HASHMAP_MOTOR);

   std::vector<std::string> joint_list;
   if(private_nh.hasParam("joint"))
   {
       private_nh.getParam("joint", joint_list);
   }
   else
   {
       ROS_WARN("No joint list!");
       joint_list.push_back("joint_0");
       joint_list.push_back("joint_1");
       private_nh.setParam("joint", joint_list);
   }

    // Initialize Joints
    for(unsigned i=0; i < joint_list.size(); ++i)
    {
        string motor_name = joint_list.at(i);
        ROS_INFO_STREAM("Motor name: " << motor_name);
        mMotor[motor_name] = new Motor(private_mNh, serial, motor_name, i);
        mMotorName.push_back(motor_name);
    }

}

bool uNavInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    ROS_INFO_STREAM("Prepare to switch!");
    return true;
}

void uNavInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    // Stop all controller in list
    for(std::list<hardware_interface::ControllerInfo>::const_iterator it = stop_list.begin(); it != stop_list.end(); ++it)
    {
        //ROS_INFO_STREAM("DO SWITCH STOP name: " << it->name << " - type: " << it->type);
        const hardware_interface::InterfaceResources& iface_res = it->claimed_resources.front();
        for (std::set<std::string>::const_iterator res_it = iface_res.resources.begin(); res_it != iface_res.resources.end(); ++res_it)
        {
            ROS_INFO_STREAM(it->name << "[" << *res_it << "] STOP");
            mMotor[*res_it]->switchController("disable");
        }
    }
    // Run all new controllers
    for(std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        //ROS_INFO_STREAM("DO SWITCH START name: " << it->name << " - type: " << it->type);
        const hardware_interface::InterfaceResources& iface_res = it->claimed_resources.front();
        for (std::set<std::string>::const_iterator res_it = iface_res.resources.begin(); res_it != iface_res.resources.end(); ++res_it)
        {
            ROS_INFO_STREAM(it->name << "[" << *res_it << "] START");
            mMotor[*res_it]->switchController(it->type);
        }
    }
}

bool uNavInterface::updateDiagnostics()
{
    if(mSerial->getStatus() == orbus::SERIAL_OK)
    {
        ROS_DEBUG_STREAM("Update diagnostic");
        // Force update all diagnostic parts
        diagnostic_updater.force_update();
        return true;
    }
    else
    {
        ROS_ERROR("Error connection! Try to connect again ...");
        if(mSerial->isAlive())
        {
            ROS_INFO("... connected!");
            // Reset list
            mSerial->resetList();
            return true;
        }
    }
    return false;
}

void uNavInterface::initializeMotors()
{
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        (*ii).second->initializeMotor();
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Initialized");
        // Send list
        mSerial->sendList();
    }
}

void uNavInterface::initializeInterfaces()
{
    // Initialize the diagnostic from the primitive object
    initializeDiagnostic();

    if (!model.initParam("/robot_description")){
      ROS_ERROR("Failed to parse urdf file");
    }
    ROS_INFO_STREAM("/robot_description found! " << model.name_ << " parsed!");

    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        /// State interface
        joint_state_interface.registerHandle(((*ii).second)->joint_state_handle);
        /// Velocity interface
        velocity_joint_interface.registerHandle(((*ii).second)->joint_handle);

        // Setup limits
        ((*ii).second)->setupLimits(model);

//        // reset position joint
//        ROS_DEBUG_STREAM("Reset position motor: " << joint[i].motor->mMotorName);
//        joint[i].motor->resetPosition(0);

        //Add motor in diagnostic updater
        diagnostic_updater.add(*((*ii).second));
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Registered");
    }

    ROS_INFO_STREAM("Send all Constraint configuration");
    // Send list of Command
    mSerial->sendList();

    /// Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
}

void uNavInterface::read(const ros::Time& time, const ros::Duration& period) {
    //ROS_DEBUG_STREAM("Get measure from uNav");
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        (*ii).second->addRequestMeasure();
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Request measures");
    }
    //Send all messages
    //serial_status = mSerial->sendList();
}

void uNavInterface::write(const ros::Time& time, const ros::Duration& period) {
    //ROS_DEBUG_STREAM("Write command to uNav");
    for( map<string, Motor*>::iterator ii=mMotor.begin(); ii!=mMotor.end(); ++ii)
    {
        (*ii).second->writeCommandsToHardware(period);
        ROS_DEBUG_STREAM("Motor [" << (*ii).first << "] Send commands");
    }
    //Send all messages
    mSerial->sendList();
}

void uNavInterface::allMotorsFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message)
{

    motor_command_map_t motor;
    motor.command_message = command;
    int number_motor = (int) motor.bitset.motor;
    ROS_DEBUG_STREAM("Frame [Option: " << option << ", HashMap: " << type << ", Nmotor: " << number_motor << ", Command: " << (int) motor.bitset.command << "]");

    if(number_motor >= 0 || number_motor < mMotorName.size())
    {
        string name = mMotorName.at(number_motor);
        // Update information
        mMotor[name]->motorFrame(option, type, motor.bitset.command, message.motor);
    }
    else
    {
        ROS_WARN_STREAM("Error enything motor is initialized for Motor: " << number_motor);
    }

}

}
