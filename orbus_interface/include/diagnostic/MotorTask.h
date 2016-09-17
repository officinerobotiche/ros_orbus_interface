#ifndef MOTORTASK_H
#define MOTORTASK_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <orbus_msgs/MotorStatus.h>

#include "hardware/SerialController.h"

typedef struct _MotorLevels
{
    double criticalCurrent; //!< Critical current motor
    double warningCurrent;  //!< Warning current motor
    double criticalTemperature; //!< Critical current motor
    double warningTemperature;  //!< Warning current motor
    //double output; //!< Output value (V)
    //double outputToll; //!< Tollerance on output value (V)
} MotorLevels;

class MotorTask : public diagnostic_updater::DiagnosticTask
{
public:
    explicit MotorTask(SerialController *serial, orbus_msgs::MotorStatus &msg, MotorLevels &levels, unsigned int number);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void updateData(motor_diagnostic_t diagnostic);

private:
    /// Command map
    motor_command_map_t command_;

    MotorLevels mlevels_;
    orbus_msgs::MotorStatus& mStatusMsg;

    SerialController *serial_;
};

#endif // MOTORTASK_H
