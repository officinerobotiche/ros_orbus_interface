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
    MotorDiagnosticConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, string path, string name, unsigned int type, unsigned int number);

    void initConfigurator();

    void setParam(motor_safety_t safety);
    /**
     * @brief getParam from ROSPARAM and save in motor_pid_t function
     * @return t return the motor_safety_t function
     */
    motor_safety_t getParam();

    MotorLevels levels;

private:
    dynamic_reconfigure::Server<orbus_interface::UnavDiagnosticConfig> *dsrv_;
    void reconfigureCB(orbus_interface::UnavDiagnosticConfig &config, uint32_t level);

    orbus_interface::UnavDiagnosticConfig last_config_, default_config_;

};

#endif // MOTORDIAGNOSTICCONFIGURATOR_H
