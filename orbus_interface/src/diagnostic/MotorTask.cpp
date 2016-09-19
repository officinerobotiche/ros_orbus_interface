#include "diagnostic/MotorTask.h"

MotorTask::MotorTask(SerialController *serial, orbus_msgs::MotorStatus &msg, MotorLevels &levels, std::string name, unsigned int number )
    : DiagnosticTask(name + "_status")
    , serial_(serial)
    , mStatusMsg(msg)
{
    mlevels_.criticalCurrent = levels.criticalCurrent;
    mlevels_.criticalTemperature = levels.criticalTemperature;
    mlevels_.warningCurrent = levels.warningCurrent;
    mlevels_.warningTemperature = levels.warningTemperature;

    // Set command message
    command_.bitset.motor = number;
    command_.bitset.command = MOTOR_DIAGNOSTIC;
}

void MotorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    serial_->addPacketSend(serial_->createPacket(command_.command_message, PACKET_REQUEST, HASHMAP_MOTOR));

    stat.add("State ", (int) mStatusMsg.state);
    stat.add("PWM rate (%)", mStatusMsg.pwm);
    stat.add("Position (deg)", ((double)mStatusMsg.position)*180.0f/M_PI);
    stat.add("Velociy (rad/s)", mStatusMsg.velocity);
    stat.add("Torque (Nm)", mStatusMsg.effort);

    stat.add("Current (A)", mStatusMsg.current);
    stat.add("Voltage (V)", mStatusMsg.voltage);
    stat.add("Watt (W)", mStatusMsg.watt);
    stat.add("Temperature (°C)", mStatusMsg.temperature);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor CONNECTED");

    if (mStatusMsg.temperature > mlevels_.criticalTemperature)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical temperature: %5.2f > %5.2f °C", mStatusMsg.temperature, mlevels_.criticalTemperature);
    }
    else if (mStatusMsg.temperature > mlevels_.warningTemperature)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature over: %5.2f > %5.2f °C", mStatusMsg.temperature, mlevels_.warningTemperature);
    }
    else
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK: %5.2f °C", mStatusMsg.temperature);
    }

    if (mStatusMsg.current > mlevels_.criticalCurrent)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical current: %5.2f > %5.2f A", mStatusMsg.current, mlevels_.criticalCurrent);
    }
    else if (mStatusMsg.current > mlevels_.warningCurrent)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Current over %5.2f > %5.2f A", mStatusMsg.current, mlevels_.warningCurrent);
    }
    else
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "Current OK: %5.2f A", mStatusMsg.current);
    }

}

void MotorTask::updateData(motor_diagnostic_t diagnostic) {

    double volt = (diagnostic.volt/1000.0f); /// in V
    double ampere = (diagnostic.current/1000.0f); /// in A

    // Update message
    mStatusMsg.header.stamp = ros::Time::now();
    mStatusMsg.watt = volt*ampere;
    mStatusMsg.current = ampere;
    mStatusMsg.voltage = volt;
    mStatusMsg.temperature = diagnostic.temperature;
}
