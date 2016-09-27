#include "configurator/MotorDiagnosticConfigurator.h"

using namespace std;

MotorDiagnosticConfigurator::MotorDiagnosticConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, string path, string name, unsigned int number)
 : GenericConfigurator(nh, serial, number)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + path + "/diagnostic/" + name;

    //Load dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<orbus_interface::UnavDiagnosticConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<orbus_interface::UnavDiagnosticConfig>::CallbackType cb = boost::bind(&MotorDiagnosticConfigurator::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void MotorDiagnosticConfigurator::reconfigureCB(orbus_interface::UnavDiagnosticConfig &config, uint32_t level)
{
    levels.critical = config.critical;
    levels.warning = config.warning;

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

    last_config_ = config;
}

