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
//#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/JointState.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <tf/transform_broadcaster.h>
//#include <tf2_ros/transform_broadcaster.h>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"

#define NUM_MOTORS 2

const std::string joint_string = "joint_states";
//const std::string odometry_string = "odometry";
//const std::string base_link_string = "base_link";
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

    //Decode a motor command
    motor_command_map_t motor_command_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    //Service
    ros::ServiceServer srv_pid, srv_parameter, srv_constraint, srv_emergency;

   // std::string tf_odometry_string_, tf_base_link_string_, tf_joint_string_;

    //motor_control_t velocity_ref[NUM_MOTORS];
    //state_controller_t status[NUM_MOTORS];

    //motor_t measure[NUM_MOTORS];
    //motor_t reference[NUM_MOTORS];

    velocity_t meas_velocity;
    //std::string name_pid;
    
    //ros::Time old_time;
    //double k_ele_left, k_ele_right;
    //sensor_msgs::JointState joint;

    void registerControlInterfaces();

    //void timerEvent(const ros::TimerEvent& event);

    bool pidServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool parameterServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool constraintServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool emergencyServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);

    bool aliveOperation(const ros::TimerEvent& event, std::vector<packet_information_t>* list_send);
    void motionPacket(const unsigned char& command, const message_abstract_u* packet);
    void motorPacket(const unsigned char& command, const message_abstract_u* packet);
    void updatePacket(std::vector<packet_information_t>* list_send);
    void addParameter(std::vector<packet_information_t>* list_send);

    motor_pid_t get_pid(std::string name);
    motor_parameter_t get_motor_parameter(std::string name);
    motor_t get_constraint(std::string name);
    motor_emergency_t get_emergency(std::string name);

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      motor_state_t state;
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() : position(0), velocity(0), effort(0), velocity_command(0) { }
    } joints_[NUM_MOTORS];

};

#endif	/* UNAVHARDWARE_H */

