#include "diagnostic/MotorTask.h"

MotorTask::MotorTask(SerialController *serial, orbus_msgs::MotorStatus &msg, MotorLevels &levels, unsigned int number )
    : DiagnosticTask("motor_status")
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
}

void MotorTask::updateData(motor_diagnostic_t diagnostic) {

    mStatusMsg.header.stamp = ros::Time::now();
    mStatusMsg.current = diagnostic.current;
    mStatusMsg.voltage = diagnostic.volt;
    mStatusMsg.temperature = diagnostic.temperature;
}
