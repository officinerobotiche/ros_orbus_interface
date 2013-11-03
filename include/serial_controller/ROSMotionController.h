/* 
 * File:   RosControllerSerial.h
 * Author: raffaello
 *
 * Created on June 7, 2013, 4:33 PM
 */

#ifndef ROSMOTIONCONTROLLER_H
#define	ROSMOTIONCONTROLLER_H

#include "std_msgs/String.h"

#include "AbstractROSController.h"

#include <serial_bridge/Enable.h>
#include <serial_bridge/Pose.h>
#include <serial_bridge/Velocity.h>
#include <serial_bridge/Motor.h>
#include <serial_bridge/Process.h>
#include <serial_bridge/Convert.h>

#include <sensor_msgs/JointState.h>
#include <serial_bridge/Update.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

const std::string update_pid_string = "update_pid";
const std::string update_parameter_string = "update_parameter";
const std::string update_constraint_string = "update_constraint";
const std::string update_process_string = "update_process";

const std::string priority_string = "priority";
const std::string frequency_string = "frequency";
const std::string parse_string = "parse_packet";

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
    ROSMotionController(std::string name_node, const ros::NodeHandle& nh, Serial* serial, ServiceSerial* service_serial);
    virtual ~ROSMotionController();

    void quit(int sig);
    void loadParameter();
    
private:
    //Initialization object
    std::string name_node_; //Name for topics, params, services
    ros::NodeHandle nh_; //NameSpace for bridge controller
    Serial* serial_; //Serial object to communicate with PIC device
    ServiceSerial* service_serial_;
    
    //Publisher communication
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
    ros::ServiceServer convert_velocity_srv_;
    //Odometry packet
    tf::TransformBroadcaster odom_broadcaster_;
    ros::Time old_time_;
    std::string tf_odometry_string_, tf_base_link_string_, tf_joint_string_;
    double positon_joint_left_, positon_joint_right_;
    std::string string_process_motion[PROCESS_MOTION_LENGTH];
    
    ros::Timer timer_;
    packet_t updatePacket();
    void timerCallback(const ros::TimerEvent& event);
    void parser(ros::Duration time, std::list<information_packet_t> serial_packet);
    
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

    pid_control_t get_pid(std::string name);
    parameter_motors_t get_parameter();
    constraint_t get_constraint();
    process_t get_process(std::string name);

    void updateOdom(const serial_bridge::Pose* pose);
    void sendJoint(serial_bridge::Motor motor_left, serial_bridge::Motor motor_right);
    void sendOdom(serial_bridge::Velocity velocity, serial_bridge::Pose pose);
    void actionAsync(packet_t packet);
};

#endif	/* ROSMOTIONCONTROLLER_H */
