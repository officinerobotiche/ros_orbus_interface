/* 
 * File:   ROSMotionController.h
 * Author: raffaello
 *
 * Created on 15 November 2013, 18:34
 */

#ifndef ROSMOTIONCONTROLLER_H
#define	ROSMOTIONCONTROLLER_H

#include "ROSController.h"

#include <ros_serial_bridge/Pose.h>
#include <ros_serial_bridge/Enable.h>
#include <ros_serial_bridge/Motor.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
//#include <tf2_ros/transform_broadcaster.h>

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

class ROSMotionController : public ROSController {
public:
    ROSMotionController(const ros::NodeHandle& nh, ParserPacket* serial);
    virtual ~ROSMotionController();

private:

    //TF transform
    tf::TransformBroadcaster odom_broadcaster;
    //tf2_ros::TransformBroadcaster odom_broadcaster;
    //Publisher communication
    ros::Publisher pub_pose;
    ros::Publisher pub_enable, pub_motor_left, pub_motor_right;
    //-Standard ROS publisher
    ros::Publisher pub_twist, pub_odom, pub_joint;
    //Subscriber
    ros::Subscriber sub_twist, sub_pose, sub_enable;
    //-Standard ROS subscriber
    ros::Subscriber sub_pose_estimate;
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

    void timerEvent(const ros::TimerEvent& event);
    
    void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void enableCallback(const ros_serial_bridge::Enable::ConstPtr &msg);
    void poseCallback(const ros_serial_bridge::Pose::ConstPtr &msg);
    void poseTFCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    bool pidServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool parameterServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    bool constraintServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    bool emergencyServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    bool aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_send);
    void motionPacket(const unsigned char& command, const abstract_message_u* packet);
    void updatePacket(std::vector<information_packet_t>* list_send);
    void addParameter(std::vector<information_packet_t>* list_send);

    void saveOdometry(const ros_serial_bridge::Pose* msg);
    void sendOdometry(const velocity_t* velocity, const ros_serial_bridge::Pose* pose);
    void sendJointState(ros_serial_bridge::Motor* motor_left, ros_serial_bridge::Motor* motor_right);

    void ConverToMotorVelocity(const geometry_msgs::Twist* msg, motor_control_t *motor_ref);

    pid_control_t get_pid(std::string name);
    parameter_motor_t get_motor_parameter(std::string name);
    parameter_unicycle_t get_unicycle_parameter();
    motor_t get_constraint(std::string name);
    emergency_t get_emergency();

};

#endif	/* ROSMOTIONCONTROLLER_H */

