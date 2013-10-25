/* 
 * File:   RosControllerSerial.h
 * Author: raffaello
 *
 * Created on June 7, 2013, 4:33 PM
 */

#ifndef ROSMOTIONCONTROLLER_H
#define	ROSMOTIONCONTROLLER_H

#include "Serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "AbstractROSController.h"

#include <boost/thread.hpp>

#include <serial_bridge/Enable.h>
#include <serial_bridge/Pose.h>
#include <serial_bridge/Velocity.h>
#include <serial_bridge/Motor.h>
#include <serial_bridge/Process.h>
#include <serial_bridge/Convert.h>
#include <serial_bridge/Service.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <serial_bridge/Update.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//const std::string name_node = "robot";

const std::string update_pid_string = "update_pid";
const std::string update_parameter_string = "update_parameter";
const std::string update_constraint_string = "update_constraint";
const std::string update_process_string = "update_process";
const std::string service_string = "service";
const std::string reset_string = "reset";
const std::string version_string = "version";
const std::string error_serial_string = "serial";

const std::string priority_string = "priority";
const std::string frequency_string = "frequency";
const std::string parse_string = "parse_packet";

const std::string command_string = "command";
const std::string measure_string = "measure";

const std::string joint_string = "joint_states";
const std::string tf_string = "tf";
const std::string velocity_string = "velocity";
const std::string enable_motors = "enable_motors";
const std::string odometry_string = "odometry";
const std::string pose_string = "pose";
const std::string motor = "motor";
const std::string process = "process";
const std::string rate_update_string = "rate";

const std::string k_ele_string = "back_EMF";
const std::string pid_string = "PID";
const std::string wheelbase_string = "wheelbase";
const std::string radius_string = "radius";
const std::string k_vel_string = "k_vel";
const std::string k_ang_string = "k_ang";
const std::string sp_min_string = "sp_min";
const std::string pwm_string = "pwm";
const std::string constraint_string = "constraint";
const std::string space_robot_string = "space_robot";
const std::string step_timer_string = "step_timer";
const std::string int_tm_mill_string = "tm_mill";
const std::string k_time_string = "k_time";
const std::string time_string = "time";
const std::string structure_string = "structure";

const std::string left_string = "Left";
const std::string right_string = "Right";
const std::string all_string = "all";
const std::string convert_string = "convert";
const std::string normalized_string = "normalized";
const std::string standard_string = "standard";

const std::string base_link_string = "base_link";

class ROSMotionController : public AbstractROSController {
public:
    ROSMotionController(std::string name_node, const ros::NodeHandle& nh, Serial* serial, int rate);
    virtual ~ROSMotionController();

    boost::thread * run();
    void setPacketStream(packet_t packet);
    void loadParameter();
    void init();
    
    void actionAsync(packet_t packet);
private:
    //Initialization object
    std::string name_node_; //Name for topics, params, services
    ros::NodeHandle nh_; //NameSpace for bridge controller
    Serial* serial_; //Serial object to comunicate with PIC device
    int rate_; //Rate to comunication with device

    //Thread control for streaming packets
    boost::thread* thr_;
    mutable boost::mutex mutex_;
    boost::condition_variable cond;
    
    //Rate for streaming topic
    ros::Rate loop_rate_;
    bool stream_exit_;
    packet_t packet_;
    //Publisher comunication
    ros::Publisher velocity_mis_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher enable_pub_;
    ros::Publisher motor_left_pub_, motor_right_pub_;
    ros::Publisher time_process_pub_;
    //-Standard ROS publisher
    ros::Publisher odom_pub_;
    ros::Publisher joint_pub_;
    //Subscriber
    ros::Subscriber velocity_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber enable_sub_;
    //-Standard ROS subscriber
    ros::Subscriber pose_estimate_sub_;
    //Service
    ros::ServiceServer pid_update_srv_, parameter_update_srv_, constraint_update_srv_, process_update_srv_;
    ros::ServiceServer convert_velocity_srv_, reset_srv_;
    //Boolean activation send packet for topic
    bool pose_active_, enable_active_, velocity_active_, velocity_mis_active_;
    bool motor_left_active_, motor_right_active_;
    bool time_process_active_;
    bool odom_active_, joint_active_;
    bool user_;
    //Odometry packet
    tf::TransformBroadcaster odom_broadcaster_;
    ros::Time old_time_;
    std::string tf_odometry_string_, tf_base_link_string_, tf_joint_string_;
    double positon_joint_left_, positon_joint_right_;
    std::string string_process_motion[PROCESS_MOTION_LENGTH];
    int pwm_motor_;

    bool stream_bool();
    void th_stream();
    packet_t updatePacket();
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    void velocityCallback(const serial_bridge::Velocity::ConstPtr &msg);
    void enableCallback(const serial_bridge::Enable::ConstPtr &msg);
    void poseCallback(const serial_bridge::Pose::ConstPtr &msg);
    void pose_tf_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    bool pid_update_Callback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&);
    bool process_update_Callback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&);
    bool parameter_update_Callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    bool constraint_update_Callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    bool convert_Callback(serial_bridge::Convert::Request &req, serial_bridge::Convert::Response &msg);
    bool service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response &msg);

    abstract_packet_t getServiceSerial(std::list<information_packet_t> configuration, unsigned char command, unsigned char service_command);
    pid_control_t get_pid(std::string name);
    parameter_t get_parameter();
    constraint_t get_constraint();
    process_t get_process(std::string name);

    float correction_time_process(float process_time);
    void updateOdom(const serial_bridge::Pose* pose);
    void sendJoint(serial_bridge::Motor motor_left, serial_bridge::Motor motor_right);
    void sendOdom(serial_bridge::Velocity velocity, serial_bridge::Pose pose);
};

#endif	/* ROSMOTIONCONTROLLER_H */
