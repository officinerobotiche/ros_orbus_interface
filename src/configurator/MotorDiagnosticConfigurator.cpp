#include "configurator/MotorDiagnosticConfigurator.h"

using namespace std;

MotorDiagnosticConfigurator::MotorDiagnosticConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, string path, string name, unsigned int type, unsigned int number)
 : GenericConfigurator(nh, serial, number)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + path + "/diagnostic/" + name;
    // Set command type
    mCommand.bitset.command = type;

    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavDiagnosticConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<orbus_interface::UnavDiagnosticConfig>::CallbackType cb = boost::bind(&MotorDiagnosticConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void MotorDiagnosticConfigurator::initConfigurator()
{
    if(mCommand.bitset.command != 0xFFFF)
    {
        /// Send configuration to board
        message_abstract_u temp;
        temp.motor.safety = getParam();
        /// Call the function in Generic Reconfigurator
        SendParameterToBoard(temp);
    }
}

motor_safety_t MotorDiagnosticConfigurator::getParam()
{
    motor_safety_t safety;

    double temp_double;
    int temp_int;
    bool temp;

    nh_.getParam(mName + "/critical", temp_double);
    safety.critical_zone = (motor_control_t) (temp_double*1000.0);
    nh_.getParam(mName + "/timeout", temp_int);
    safety.timeout = (uint32_t) temp_int;
    nh_.getParam(mName + "/autorestore", temp_int);
    safety.autorestore = (uint32_t) temp_int;

    ROS_DEBUG_STREAM("Read safety from " << mName << " [critical:" << safety.critical_zone << ", Timeout:" << (int) safety.timeout << ", Autorestore:" << (int) safety.autorestore << "]");

    return safety;
}


void MotorDiagnosticConfigurator::reconfigureCB(orbus_interface::UnavDiagnosticConfig &config, uint32_t level)
{
    motor_safety_t safety;

    levels.critical = config.critical;
    levels.warning = config.warning;
    safety.critical_zone = (motor_control_t) (config.critical*1000.0);
    safety.timeout = (uint32_t) config.timeout;
    safety.autorestore = (uint32_t) config.autorestore;

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = last_config_;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(mCommand.bitset.command != 0xFFFF)
    {
        /// Send configuration to board
        message_abstract_u temp;
        temp.motor.safety = safety;
        /// Call the function in Generic Reconfigurator
        SendParameterToBoard(temp);
    }

    last_config_ = config;
}

