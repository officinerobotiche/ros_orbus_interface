#include "hardware/Motor.h"

Motor::Motor(unsigned int number) : DiagnosticTask("motor_" + to_string(number) + "_status")
{
    mName = "motor_" + to_string(number);

}

void Motor::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

}
