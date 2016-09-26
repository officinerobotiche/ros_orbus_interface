#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

using namespace std;

class Motor : public diagnostic_updater::DiagnosticTask
{
public:
    explicit Motor(unsigned int number);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
    string mName;
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;

    ros::Publisher diagnostic_publisher;
};

#endif // MOTOR_H
