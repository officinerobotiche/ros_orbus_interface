#ifndef MOTORDIAGNOSTICCONFIGURATOR_H
#define MOTORDIAGNOSTICCONFIGURATOR_H

#include "configurator/GenericConfigurator.h"

#include <orbus_interface/UnavDiagnosticConfig.h>

using namespace std;

typedef struct _MotorLevels
{
    double critical; //!< Critical
    double warning;  //!< Warning
} MotorLevels;

class MotorDiagnosticConfigurator : public GenericConfigurator
{
public:
    MotorDiagnosticConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, string path, string name, unsigned int number);

    //void initConfigurator();

    MotorLevels levels;

private:
    dynamic_reconfigure::Server<orbus_interface::UnavDiagnosticConfig> *dsrv_;
    void reconfigureCB(orbus_interface::UnavDiagnosticConfig &config, uint32_t level);

    orbus_interface::UnavDiagnosticConfig last_config_, default_config_;

};

#endif // MOTORDIAGNOSTICCONFIGURATOR_H
