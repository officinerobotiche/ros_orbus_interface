/* 
 * File:   ROSMotionController.h
 * Author: raffaello
 *
 * Created on 15 November 2013, 18:34
 */

#ifndef ROSMOTIONCONTROLLER_H
#define	ROSMOTIONCONTROLLER_H

#include "ROSController.h"

#include <serial_bridge/Pose.h>
#include <serial_bridge/Enable.h>
#include <serial_bridge/Motor.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

const std::string joint_string = "joint_states";
const std::string odometry_string = "odometry";
const std::string base_link_string = "base_link";

const std::string wheelbase_string = "wheelbase";
const std::string radius_string = "radius";

class ROSMotionController : public ROSController {
public:
    ROSMotionController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial);
    virtual ~ROSMotionController();

private:

    //TF transform
    tf::TransformBroadcaster odom_broadcaster;
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
    ros::ServiceServer srv_pid, srv_parameter, srv_constraint;

    double pwm_motor;
    std::string tf_odometry_string_, tf_base_link_string_, tf_joint_string_;

    geometry_msgs::Twist twist;
    velocity_t meas_velocity;
    serial_bridge::Pose pose;
    serial_bridge::Motor motor_left, motor_right;
    serial_bridge::Enable enable_motors;
    std::string name_pid;
    
    bool alive_operation;
    ros::Timer delay_timer_;
    ros::Time old_time;
    double k_ele_left, k_ele_right;
    double positon_joint_left, positon_joint_right;
    sensor_msgs::JointState joint;

    void timerStopCallback(const ros::TimerEvent& event);
    void timerEvent(const ros::TimerEvent& event);

    void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void enableCallback(const serial_bridge::Enable::ConstPtr &msg);
    void poseCallback(const serial_bridge::Pose::ConstPtr &msg);
    void poseTFCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    bool pidServiceCallback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&);
    bool parameterServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    bool constraintServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    bool aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_send);
    void motionPacket(const unsigned char& command, const abstract_packet_t* packet);
    void updatePacket(std::vector<information_packet_t>* list_send);
    void addParameter(std::vector<information_packet_t>* list_send);

    void saveOdometry(const serial_bridge::Pose* msg);
    void sendOdometry(const velocity_t* velocity, const serial_bridge::Pose* pose);
    void sendJointState(serial_bridge::Motor* motor_left, serial_bridge::Motor* motor_right);

    pid_control_t get_pid(std::string name);
    parameter_motors_t get_parameter();
    constraint_t get_constraint();

};

#endif	/* ROSMOTIONCONTROLLER_H */

