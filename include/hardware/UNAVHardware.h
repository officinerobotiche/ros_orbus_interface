/* 
 * File:   UNAVHardware.h
 * Author: raffaello
 *
 * Created on 15 November 2013, 18:34
 */

#ifndef UNAVHARDWARE_H
#define	UNAVHARDWARE_H

#include "ORBHardware.h"

#include <ros_serial_bridge/Pose.h>
#include <ros_serial_bridge/Enable.h>
#include <ros_serial_bridge/Motor.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
//#include <tf2_ros/transform_broadcaster.h>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"

#define NUM_MOTORS 2
#define REF_MOTOR_LEFT 0
#define REF_MOTOR_RIGHT 1

const std::string joint_string = "joint_states";
const std::string odometry_string = "odometry";
const std::string base_link_string = "base_link";
const std::string paramenter_unicycle_string = "unicycle";
const std::string emergency_string = "emergency";

const std::string wheelbase_string = "wheelbase";
const std::string radius_string = "radius";

class UNAVHardware : public ORBHardware {
public:
    UNAVHardware(const ros::NodeHandle& nh, ParserPacket* serial);
    virtual ~UNAVHardware();

    void updateJointsFromHardware();
    void writeCommandsToHardware();

private:

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    //Service
    ros::ServiceServer srv_pid, srv_parameter, srv_constraint, srv_emergency;

    std::string tf_odometry_string_, tf_base_link_string_, tf_joint_string_;

    motor_control_t velocity_ref[NUM_MOTORS];
    state_controller_t status[NUM_MOTORS];

    motor_t measure[NUM_MOTORS];
    motor_t reference[NUM_MOTORS];

    geometry_msgs::Twist twist;
    velocity_t meas_velocity;
    ros_serial_bridge::Pose pose;
    //ros_serial_bridge::Motor motor_left, motor_right;
    ros_serial_bridge::Enable enable_motors;
    std::string name_pid;
    
    ros::Time old_time;
    double k_ele_left, k_ele_right;
    double positon_joint_left, positon_joint_right;
    sensor_msgs::JointState joint;

    void registerControlInterfaces();

    void timerEvent(const ros::TimerEvent& event);

    bool pidServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool parameterServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool constraintServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool emergencyServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    bool aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_send);
    void motionPacket(const unsigned char& command, const abstract_message_u* packet);
    void updatePacket(std::vector<information_packet_t>* list_send);
    void addParameter(std::vector<information_packet_t>* list_send);

    void ConverToMotorVelocity(const geometry_msgs::Twist* msg, motor_control_t *motor_ref);

    pid_control_t get_pid(std::string name);
    parameter_motor_t get_motor_parameter(std::string name);
    parameter_unicycle_t get_unicycle_parameter();
    motor_t get_constraint(std::string name);
    emergency_t get_emergency();

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() : position(0), velocity(0), effort(0), velocity_command(0) { }
    } joints_[NUM_MOTORS];

};

#endif	/* UNAVHARDWARE_H */

