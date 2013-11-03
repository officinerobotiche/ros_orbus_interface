/* 
 * File:   RosControllerSerial.cpp
 * Author: Raffaello Bonghi
 * 
 * Created on June 7, 2013, 4:33 PM
 */

#include "serial_controller/ROSMotionController.h"

/*
 *
 */
ROSMotionController::ROSMotionController(std::string name_node, const ros::NodeHandle& nh, Serial* serial, ServiceSerial* service_serial)
: nh_(nh) {
    name_node_ = name_node; // Initialize node name
    this->serial_ = serial; // Initialize serial port
    service_serial_ = service_serial;
    serial_->asyncPacket(&ROSMotionController::actionAsync, this);

    //Open Publisher
    //- Measure
    velocity_mis_pub_ = nh_.advertise<serial_bridge::Velocity>("/" + name_node + "/" + measure_string + "/" + velocity_string, 1000,
            boost::bind(&ROSMotionController::connectCallback, this, _1));
    //- Command receive
    pose_pub_ = nh_.advertise<serial_bridge::Pose>("/" + name_node + "/" + pose_string, 1000,
            boost::bind(&ROSMotionController::connectCallback, this, _1));
    velocity_pub_ = nh_.advertise<serial_bridge::Velocity>("/" + name_node + "/" + velocity_string, 1000,
            boost::bind(&ROSMotionController::connectCallback, this, _1));
    enable_pub_ = nh_.advertise<serial_bridge::Enable>("/" + name_node + "/" + enable_motors, 1000,
            boost::bind(&ROSMotionController::connectCallback, this, _1));
    motor_left_pub_ = nh_.advertise<serial_bridge::Motor>("/" + name_node + "/" + motor + "/" + left_string, 1000,
            boost::bind(&ROSMotionController::connectCallback, this, _1));
    motor_right_pub_ = nh_.advertise<serial_bridge::Motor>("/" + name_node + "/" + motor + "/" + right_string, 1000,
            boost::bind(&ROSMotionController::connectCallback, this, _1));
    time_process_pub_ = nh_.advertise<serial_bridge::Process>("/" + name_node + "/" + process, 1000,
            boost::bind(&ROSMotionController::connectCallback, this, _1));
    //-- Conventional (Using TF, NAV)
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/" + name_node + "/" + odometry_string, 1000,
            boost::bind(&ROSMotionController::connectCallback, this, _1));
    //JointState position
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_string, 1,
            boost::bind(&ROSMotionController::connectCallback, this, _1));

    //Open Subscriber
    //- Command
    pose_sub_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + pose_string, 1, &ROSMotionController::poseCallback, this);
    velocity_sub_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + velocity_string, 1, &ROSMotionController::velocityCallback, this);
    enable_sub_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + enable_motors, 1, &ROSMotionController::enableCallback, this);
    //-- Conventional (Using TF, NAV)
    pose_estimate_sub_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + odometry_string, 1, &ROSMotionController::pose_tf_Callback, this);

    //Open Service
    pid_update_srv_ = nh_.advertiseService("/" + name_node + "/" + update_pid_string, &ROSMotionController::pid_update_Callback, this);
    parameter_update_srv_ = nh_.advertiseService("/" + name_node + "/" + update_parameter_string, &ROSMotionController::parameter_update_Callback, this);
    constraint_update_srv_ = nh_.advertiseService("/" + name_node + "/" + update_constraint_string, &ROSMotionController::constraint_update_Callback, this);
    process_update_srv_ = nh_.advertiseService("/" + name_node + "/" + update_process_string, &ROSMotionController::process_update_Callback, this);
    convert_velocity_srv_ = nh_.advertiseService("/" + name_node + "/" + convert_string, &ROSMotionController::convert_Callback, this);

    //Initialize string motion
    string_process_motion[PROCESS_PID_LEFT] = PID_LEFT_STRING;
    string_process_motion[PROCESS_PID_RIGHT] = PID_RIGHT_STRING;
    string_process_motion[PROCESS_VELOCITY] = VELOCITY_STRING;
    string_process_motion[PROCESS_ODOMETRY] = ODOMETRY_STRING;

    positon_joint_left_ = 0;
    positon_joint_right_ = 0;

    double rate = 1;
    if (nh_.hasParam(name_node_ + "/" + rate_update_string)) {
        nh_.getParam("/" + name_node_ + "/" + rate_update_string, rate);
        ROS_INFO("Sync parameter %s: load - %f Hz", rate_update_string.c_str(), rate);
        //ROS_INFO("TODO Sync %s", rate_update_string.c_str());
    } else {
        nh_.setParam("/" + name_node_ + "/" + rate_update_string, rate);
        ROS_INFO("Sync parameter %s: set - %f Hz", rate_update_string.c_str(), rate);
    }
    //Timer
    timer_ = nh_.createTimer(ros::Duration(1/rate), &ROSMotionController::timerCallback, this, false, false);
}

void ROSMotionController::actionAsync(packet_t packet) {
    ROS_INFO("ROS Motion Controller Async");
}

ROSMotionController::~ROSMotionController() {
}

void ROSMotionController::loadParameter() {
    packet_t send_pkg;
    send_pkg.length = 0;
    //Constraint
    if (nh_.hasParam(name_node_ + "/" + joint_string + "/" + constraint_string)) {
        ROS_DEBUG("Sync parameter %s: ROS -> ROBOT", constraint_string.c_str());
        constraint_t constraint = get_constraint();
        Serial::addPacket(&send_pkg, CONSTRAINT, CHANGE, (abstract_packet_t*) & constraint);
    } else {
        ROS_DEBUG("Sync parameter %s: ROBOT -> ROS", constraint_string.c_str());
        Serial::addPacket(&send_pkg, CONSTRAINT, REQUEST, NULL);
    }
    //Load PID/Left
    if (nh_.hasParam(name_node_ + +"/" + pid_string + "/" + left_string)) {
        ROS_DEBUG("Sync parameter %s/%s: ROS -> ROBOT", pid_string.c_str(), left_string.c_str());
        pid_control_t pid_l = get_pid(left_string);
        Serial::addPacket(&send_pkg, PID_CONTROL_L, CHANGE, (abstract_packet_t*) & pid_l);
    } else {
        ROS_DEBUG("Sync parameter %s/%s: ROBOT -> ROS", pid_string.c_str(), left_string.c_str());
        Serial::addPacket(&send_pkg, PID_CONTROL_L, REQUEST, NULL);
    }
    //Load PID/Right
    if (nh_.hasParam(name_node_ + +"/" + pid_string + "/" + right_string)) {
        ROS_DEBUG("Sync parameter %s/%s: ROS -> ROBOT", pid_string.c_str(), right_string.c_str());
        pid_control_t pid_r = get_pid(right_string);
        Serial::addPacket(&send_pkg, PID_CONTROL_R, CHANGE, (abstract_packet_t*) & pid_r);
    } else {
        ROS_DEBUG("Sync parameter %s/%s: ROBOT -> ROS", pid_string.c_str(), right_string.c_str());
        Serial::addPacket(&send_pkg, PID_CONTROL_R, REQUEST, NULL);
    }
    //Parameter motors
    if (nh_.hasParam(name_node_ + "/" + structure_string)) {
        ROS_DEBUG("Sync parameter %s: ROS -> ROBOT", structure_string.c_str());
        parameter_motors_t parameter_motors = get_parameter();
        Serial::addPacket(&send_pkg, PARAMETER_MOTORS, CHANGE, (abstract_packet_t*) & parameter_motors);
    } else {
        ROS_DEBUG("Sync parameter %s: ROBOT -> ROS", structure_string.c_str());
        Serial::addPacket(&send_pkg, PARAMETER_MOTORS, REQUEST, NULL);
    }
    //Parameter frequency
    if (nh_.hasParam(name_node_ + "/" + frequency_string)) {
        ROS_DEBUG("Sync parameter %s: ROS -> ROBOT", frequency_string.c_str());
        ROS_INFO("TODO Sync %s", frequency_string.c_str());
    } else {
        ROS_DEBUG("Sync parameter %s: ROBOT -> ROS", frequency_string.c_str());
        Serial::addPacket(&send_pkg, FRQ_PROCESS, REQUEST, NULL);
    }
    //Parameter priority
    if (nh_.hasParam(name_node_ + "/" + priority_string)) {
        ROS_DEBUG("Sync parameter %s: ROS -> ROBOT", priority_string.c_str());
        ROS_INFO("TODO Sync %s", priority_string.c_str());
    } else {
        ROS_DEBUG("Sync parameter %s: ROBOT -> ROS", priority_string.c_str());
        Serial::addPacket(&send_pkg, PRIORITY_PROCESS, REQUEST, NULL);
    }
    //Names TF
    if (nh_.hasParam(name_node_ + "/" + tf_string)) {
        ROS_DEBUG("Sync parameter %s: load", tf_string.c_str());
        nh_.param<std::string>("/" + name_node_ + "/" + tf_string + "/" + odometry_string, tf_odometry_string_, tf_odometry_string_);
        nh_.param<std::string>("/" + name_node_ + "/" + tf_string + "/" + base_link_string, tf_base_link_string_, tf_base_link_string_);
        nh_.param<std::string>("/" + name_node_ + "/" + tf_string + "/" + joint_string, tf_joint_string_, tf_joint_string_);
    } else {
        ROS_DEBUG("Sync parameter %s: set", tf_string.c_str());
        tf_odometry_string_ = odometry_string;
        tf_base_link_string_ = base_link_string;
        tf_joint_string_ = joint_string;
        nh_.setParam("/" + name_node_ + "/" + tf_string + "/" + odometry_string, tf_odometry_string_);
        nh_.setParam("/" + name_node_ + "/" + tf_string + "/" + base_link_string, tf_base_link_string_);
        nh_.setParam("/" + name_node_ + "/" + tf_string + "/" + joint_string, tf_joint_string_);
    }
    //Names ele
    if (nh_.hasParam(name_node_ + "/" + joint_string + "/" + k_ele_string)) {
        ROS_DEBUG("Sync parameter %s/%s: load", joint_string.c_str(), k_ele_string.c_str());
        ROS_INFO("TODO Sync %s/%s", joint_string.c_str(), k_ele_string.c_str());
    } else {
        ROS_DEBUG("Sync parameter %s/%s: set", joint_string.c_str(), k_ele_string.c_str());
        nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_ele_string + "/" + left_string, 1.0);
        nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_ele_string + "/" + right_string, 1.0);
    }
    // total space move robot
    nh_.setParam("/" + name_node_ + "/" + space_robot_string, 0);

    if (send_pkg.length > 0) {
        packet_t packet = serial_->sendPacket(send_pkg);
        double rate = ((double) packet.time) / 1000000000;
        //Parsing packets
        parser(ros::Duration(rate), Serial::parsing(NULL, packet));
    }
}

void ROSMotionController::timerCallback(const ros::TimerEvent& event) {
    packet_t send_packet = updatePacket();
    double rate = 1;
    nh_.getParam("/" + name_node_ + "/" + rate_update_string, rate);
    timer_.setPeriod(ros::Duration(1/rate));
    if (send_packet.length == 0) {
        ROS_INFO("Wait user");
        timer_.stop();
    } else {
        //ROS_INFO("Start streaming");
        packet_t receive_packet = serial_->sendPacket(send_packet);
        double rate = ((double) receive_packet.time) / 1000000000;
        //Parsing packets
        parser(ros::Duration(rate), Serial::parsing(NULL, receive_packet));
    }
}

packet_t ROSMotionController::updatePacket() {
    packet_t packet;
    packet.length = 0;
    std::string packet_string;
    if ((pose_pub_.getNumSubscribers() >= 1) || (odom_pub_.getNumSubscribers() >= 1)) {
        packet_string += "Odo ";
        Serial::addPacket(&packet, COORDINATE, REQUEST, NULL);
    }
    if (velocity_pub_.getNumSubscribers() >= 1) {
        packet_string += "Vel ";
        Serial::addPacket(&packet, VELOCITY, REQUEST, NULL);
    }
    if (enable_pub_.getNumSubscribers() >= 1) {
        packet_string += "Ena ";
        Serial::addPacket(&packet, ENABLE, REQUEST, NULL);
    }
    if ((velocity_mis_pub_.getNumSubscribers() >= 1) || (odom_pub_.getNumSubscribers() >= 1)) {
        packet_string += "VelMis ";
        Serial::addPacket(&packet, VELOCITY_MIS, REQUEST, NULL);
    }
    if ((motor_left_pub_.getNumSubscribers() >= 1) || (joint_pub_.getNumSubscribers() >= 1)) {
        packet_string += "MotL ";
        Serial::addPacket(&packet, MOTOR_L, REQUEST, NULL);
    }
    if ((motor_right_pub_.getNumSubscribers() >= 1) || (joint_pub_.getNumSubscribers() >= 1)) {
        packet_string += "MotR ";
        Serial::addPacket(&packet, MOTOR_R, REQUEST, NULL);
    }
    if (time_process_pub_.getNumSubscribers() >= 1) {
        packet_string += "Proc ";
        Serial::addPacket(&packet, TIME_PROCESS, REQUEST, NULL);
    }
    //ROS_INFO("[ %s]", packet_string.c_str());
    return packet;
}

void ROSMotionController::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
    timer_.start();
}

void ROSMotionController::velocityCallback(const serial_bridge::Velocity::ConstPtr &msg) {
    //  ROS_INFO("VELOCITY CALLBACK");
    packet_t send_pkg;
    send_pkg.length = 0;
    velocity_t velocity;
    velocity.v = msg->lin_vel;
    velocity.w = msg->ang_vel;
    Serial::addPacket(&send_pkg, VELOCITY, CHANGE, (abstract_packet_t*) & velocity);
    // TODO VERIFY THE PACKET
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
}

void ROSMotionController::enableCallback(const serial_bridge::Enable::ConstPtr &msg) {
    packet_t send_pkg;
    send_pkg.length = 0;
    enable_motor_t enable = msg->enable;
    Serial::addPacket(&send_pkg, ENABLE, CHANGE, (abstract_packet_t*) & enable);
    // TODO VERIFY THE PACKET
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
}

void ROSMotionController::updateOdom(const serial_bridge::Pose* pose) {
    float space;
    packet_t packet;
    packet.length = 0;
    Serial::addPacket(&packet, COORDINATE, REQUEST, NULL);
    std::list<information_packet_t> configuration_robot;
    configuration_robot = Serial::parsing(NULL, serial_->sendPacket(packet));
    for (std::list<information_packet_t>::iterator list_iter = configuration_robot.begin(); list_iter != configuration_robot.end(); list_iter++) {
        information_packet_t packet = (*list_iter);
        if (packet.option == CHANGE) {
            switch (packet.command) {
                case COORDINATE:
                    space = packet.packet.coordinate.space;
                    break;
            }
        }
    }
    positon_joint_left_ = 0;
    positon_joint_right_ = 0;
    packet_t send_pkg;
    send_pkg.length = 0;
    coordinate_t coordinate;
    coordinate.x = pose->x;
    coordinate.y = pose->y;
    coordinate.theta = pose->theta;
    coordinate.space = space;
    Serial::addPacket(&send_pkg, COORDINATE, CHANGE, (abstract_packet_t*) & coordinate);
    // TODO VERIFY THE PACKET
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
}

void ROSMotionController::pose_tf_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    serial_bridge::Pose pose;
    pose.x = msg.get()->pose.pose.position.x;
    pose.y = msg.get()->pose.pose.position.y;
    pose.theta = tf::getYaw(msg.get()->pose.pose.orientation);
    updateOdom(&pose);
    ROS_INFO("Update initial position: [x: %f, y: %f, th: %f]", pose.x, pose.y, pose.theta);
}

void ROSMotionController::poseCallback(const serial_bridge::Pose::ConstPtr &msg) {
    //  ROS_INFO("POSE CALLBACK");
    updateOdom(msg.get());
}

pid_control_t ROSMotionController::get_pid(std::string name) {
    pid_control_t pid;
    double temp;
    std::string name_pid = "/" + name_node_ + "/" + pid_string + "/" + name + "/";
    nh_.getParam(name_pid + "P", temp);
    pid.kp = temp;
    nh_.getParam(name_pid + "I", temp);
    pid.ki = temp;
    nh_.getParam(name_pid + "D", temp);
    pid.kd = temp;
    return pid;
}

parameter_motors_t ROSMotionController::get_parameter() {
    parameter_motors_t parameter;
    double temp;
    int temp_int;

    nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + wheelbase_string, temp);
    parameter.wheelbase = temp;
    nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + right_string, temp);
    parameter.radius_r = temp;
    nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + left_string, temp);
    parameter.radius_l = temp;
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + k_vel_string + "/" + right_string, temp);
    parameter.k_vel_r = temp;
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + k_vel_string + "/" + left_string, temp);
    parameter.k_vel_l = temp;
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + k_ang_string + "/" + right_string, temp);
    parameter.k_ang_r = temp;
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + k_ang_string + "/" + left_string, temp);
    parameter.k_ang_l = temp;
    nh_.getParam("/" + name_node_ + "/" + sp_min_string, temp);
    parameter.sp_min = temp;
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + pwm_string, temp_int);
    parameter.pwm_step = temp_int;
    return parameter;
}

constraint_t ROSMotionController::get_constraint() {
    constraint_t constraint;
    double temp;

    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + constraint_string + "/" + right_string, temp);
    constraint.max_right = temp;
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + constraint_string + "/" + left_string, temp);
    constraint.max_left = temp;
    return constraint;
}

process_t ROSMotionController::get_process(std::string name) {
    process_t process;
    int temp;
    process.idle = 0;
    for (int i = 0; i < PROCESS_MOTION_LENGTH; i++) {
        nh_.getParam("/" + name_node_ + "/" + name + "/" + string_process_motion[i], temp);
        process.process[i] = temp;
    }
    if (name.compare(priority_string) == 0) {
        nh_.getParam("/" + name_node_ + "/" + name + "/" + parse_string, temp);
        process.parse_packet = temp;
    } else process.parse_packet = 0;
    return process;
}

bool ROSMotionController::parameter_update_Callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    packet_t send_pkg;
    send_pkg.length = 0;
    parameter_motors_t parameter = get_parameter();
    Serial::addPacket(&send_pkg, PARAMETER_MOTORS, CHANGE, (abstract_packet_t*) & parameter);
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    return true;
}

bool ROSMotionController::process_update_Callback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&) {
    std::string name = req.name;
    process_t process;
    packet_t send_pkg;
    send_pkg.length = 0;
    ROS_INFO("PROCESS UPDATE");
    if ((name.compare(priority_string) == 0) || (name.compare(all_string) == 0)) {
        process = get_process(priority_string);
        Serial::addPacket(&send_pkg, PRIORITY_PROCESS, CHANGE, (abstract_packet_t*) & process);
    }
    if ((name.compare(frequency_string) == 0) || (name.compare(all_string) == 0)) {
        process = get_process(frequency_string);
        Serial::addPacket(&send_pkg, FRQ_PROCESS, CHANGE, (abstract_packet_t*) & process);
    }
    if (send_pkg.length != 0) {
        // TODO VERIFY THE PACKET
        Serial::parsing(NULL, serial_->sendPacket(send_pkg));
        return true;
    } else {
        ROS_ERROR("PROCESS ERROR UPDATE");
        return false;
    }
}

bool ROSMotionController::pid_update_Callback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&) {
    std::string name = req.name;
    pid_control_t pid;
    packet_t send_pkg;
    send_pkg.length = 0;
    ROS_INFO("PID UPDATE");
    if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0)) {
        pid = get_pid(left_string);
        Serial::addPacket(&send_pkg, PID_CONTROL_L, CHANGE, (abstract_packet_t*) & pid);
    }
    if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0)) {
        pid = get_pid(right_string);
        Serial::addPacket(&send_pkg, PID_CONTROL_R, CHANGE, (abstract_packet_t*) & pid);
    }
    if (send_pkg.length != 0) {
        // TODO VERIFY THE PACKET
        Serial::parsing(NULL, serial_->sendPacket(send_pkg));
        return true;
    } else {
        ROS_ERROR("PID ERROR UPDATE");
        return false;
    }
}

bool ROSMotionController::constraint_update_Callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    packet_t send_pkg;
    send_pkg.length = 0;
    constraint_t constraint = get_constraint();
    Serial::addPacket(&send_pkg, CONSTRAINT, CHANGE, (abstract_packet_t*) & constraint);
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    return true;
}

bool ROSMotionController::convert_Callback(serial_bridge::Convert::Request &req, serial_bridge::Convert::Response & msg) {
    double wheelbase, radius_r, radius_l;
    nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + wheelbase_string, wheelbase);
    if (req.type.compare(normalized_string) == 0) {
        msg.lin_vel = (req.vel_r + req.vel_l) / 2;
        msg.ang_vel = (req.vel_r - req.vel_l) / wheelbase;
        return true;
    } else if (req.type.compare(standard_string) == 0) {
        nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + right_string, radius_r);
        nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + left_string, radius_l);
        msg.lin_vel = (radius_r * req.vel_r + radius_l * req.vel_l) / 2;
        msg.ang_vel = (radius_r * req.vel_r - radius_l * req.vel_l) / wheelbase;
        return true;
    } else return false;

}

void ROSMotionController::parser(ros::Duration duration, std::list<information_packet_t> serial_packet) {
    serial_bridge::Pose pose;
    serial_bridge::Velocity velocity;
    serial_bridge::Motor motor_left, motor_right;
    serial_bridge::Enable enable_motors;
    serial_bridge::Process time_process;
    std::string name_pid;
    int pwm_motor = 0;
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + pwm_string, pwm_motor);
    for (std::list<information_packet_t>::iterator list_iter = serial_packet.begin(); list_iter != serial_packet.end(); list_iter++) {
        information_packet_t packet = (*list_iter);
        //ROS_INFO("Command %c - Option %c", packet.command, packet.option);
        if (packet.option == CHANGE) {
            //ROS publish packets
            ros::spinOnce();
            //Decode commands
            switch (packet.command) {
                case PRIORITY_PROCESS:
                    for (int i = 0; i < PROCESS_MOTION_LENGTH; i++) {
                        nh_.setParam("/" + name_node_ + "/" + priority_string + "/" + string_process_motion[i], packet.packet.process.process[i]);
                    }
                    nh_.setParam("/" + name_node_ + "/" + priority_string + "/" + parse_string, packet.packet.process.parse_packet);
                    break;
                case FRQ_PROCESS:
                    for (int i = 0; i < PROCESS_MOTION_LENGTH; i++) {
                        nh_.setParam("/" + name_node_ + "/" + frequency_string + "/" + string_process_motion[i], packet.packet.process.process[i]);
                    }
                    break;
                case CONSTRAINT:
                    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + constraint_string + "/" + right_string, packet.packet.constraint.max_right);
                    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + constraint_string + "/" + left_string, packet.packet.constraint.max_left);
                    break;
                case PARAMETER_MOTORS:
                    nh_.setParam("/" + name_node_ + "/" + structure_string + "/" + wheelbase_string, packet.packet.parameter_motors.wheelbase);
                    nh_.setParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + right_string, packet.packet.parameter_motors.radius_r);
                    nh_.setParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + left_string, packet.packet.parameter_motors.radius_l);
                    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_vel_string + "/" + right_string, packet.packet.parameter_motors.k_vel_r);
                    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_vel_string + "/" + left_string, packet.packet.parameter_motors.k_vel_l);
                    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_ang_string + "/" + right_string, packet.packet.parameter_motors.k_ang_r);
                    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_ang_string + "/" + left_string, packet.packet.parameter_motors.k_ang_l);
                    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + pwm_string, packet.packet.parameter_motors.pwm_step);
                    nh_.setParam("/" + name_node_ + "/" + sp_min_string, packet.packet.parameter_motors.sp_min);
                    break;
                case PID_CONTROL_L:
                    name_pid = "/" + name_node_ + "/" + pid_string + "/" + left_string + "/";
                    nh_.setParam(name_pid + "P", packet.packet.pid.kp);
                    nh_.setParam(name_pid + "I", packet.packet.pid.ki);
                    nh_.setParam(name_pid + "D", packet.packet.pid.kd);
                    break;
                case PID_CONTROL_R:
                    name_pid = "/" + name_node_ + "/" + pid_string + "/" + right_string + "/";
                    nh_.setParam(name_pid + "P", packet.packet.pid.kp);
                    nh_.setParam(name_pid + "I", packet.packet.pid.ki);
                    nh_.setParam(name_pid + "D", packet.packet.pid.kd);
                    break;
                case TIME_PROCESS:
                    time_process.time = duration;
                    time_process.idle = service_serial_->getTimeProcess(packet.packet.process.idle);
                    time_process.parse_packet = service_serial_->getTimeProcess(packet.packet.process.parse_packet);
                    time_process.pid_l = service_serial_->getTimeProcess(packet.packet.process.process[0]);
                    time_process.pid_r = service_serial_->getTimeProcess(packet.packet.process.process[1]);
                    time_process.velocity = service_serial_->getTimeProcess(packet.packet.process.process[2]);
                    time_process.dead_reckoning = service_serial_->getTimeProcess(packet.packet.process.process[3]);
                    time_process_pub_.publish(time_process);
                    break;
                case MOTOR_L:
                    motor_left.time = duration;
                    motor_left.rifer_vel = ((double) packet.packet.motor.rifer_vel) / 1000;
                    motor_left.control_vel = -((double) packet.packet.motor.control_vel - ((double) pwm_motor) / 2) / (((double) pwm_motor) / 2);
                    motor_left.measure_vel = ((double) packet.packet.motor.measure_vel) / 1000;
                    motor_left.current = ((double) packet.packet.motor.current) / 1000;
                    motor_left_pub_.publish(motor_left);
                    break;
                case MOTOR_R:
                    motor_right.time = duration;
                    motor_right.rifer_vel = ((double) packet.packet.motor.rifer_vel) / 1000;
                    motor_right.control_vel = ((double) packet.packet.motor.control_vel - ((double) pwm_motor) / 2) / (((double) pwm_motor) / 2);
                    motor_right.measure_vel = ((double) packet.packet.motor.measure_vel) / 1000;
                    motor_right.current = ((double) packet.packet.motor.current) / 1000;
                    motor_right_pub_.publish(motor_right);
                    break;
                case COORDINATE:
                    pose.time = duration;
                    pose.x = packet.packet.coordinate.x;
                    pose.y = packet.packet.coordinate.y;
                    pose.theta = packet.packet.coordinate.theta;
                    nh_.setParam("/" + name_node_ + "/" + space_robot_string, packet.packet.coordinate.space);
                    //              ROS_INFO("COORDINATE");
                    pose_pub_.publish(pose);
                    break;
                case VELOCITY:
                    velocity.time = duration;
                    velocity.lin_vel = packet.packet.velocity.v;
                    velocity.ang_vel = packet.packet.velocity.w;
                    //              ROS_INFO("VELOCITY");
                    velocity_pub_.publish(velocity);
                    break;
                case VELOCITY_MIS:
                    velocity.time = duration;
                    velocity.lin_vel = packet.packet.velocity.v;
                    velocity.ang_vel = packet.packet.velocity.w;
                    //              ROS_INFO("VELOCITY_MIS");
                    velocity_mis_pub_.publish(velocity);
                    break;
                case ENABLE:
                    enable_motors.time = duration;
                    enable_motors.enable = packet.packet.enable;
                    //ROS_INFO("ENABLE");
                    enable_pub_.publish(enable_motors);
                    break;
            }
        }
    }
    // Send Odometry message
    if (odom_pub_.getNumSubscribers() >= 1) {
        //        ROS_INFO("SEND ODOM");
        sendOdom(velocity, pose);
        //ROS publish packets
        ros::spinOnce();
    }
    if (joint_pub_.getNumSubscribers() >= 1) {
        sendJoint(motor_left, motor_right);
        //ROS publish packets
        ros::spinOnce();
    }
}

void ROSMotionController::sendJoint(serial_bridge::Motor motor_left, serial_bridge::Motor motor_right) {
    sensor_msgs::JointState joint;
    ros::Time now = ros::Time::now();
    double rate = (now - old_time_).toSec();
    old_time_ = now;
    double k_ele_left, k_ele_right;
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + k_ele_string + "/" + left_string, k_ele_left);
    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + k_ele_string + "/" + right_string, k_ele_right);

    joint.header.frame_id = tf_joint_string_;
    joint.header.stamp = old_time_;
    joint.name.resize(2);
    joint.effort.resize(2);
    joint.velocity.resize(2);
    joint.position.resize(2);
    joint.name[0] = left_string;
    joint.name[1] = right_string;
    joint.velocity[0] = motor_left.measure_vel;
    joint.velocity[1] = motor_right.measure_vel;
    positon_joint_left_ = fmod(positon_joint_left_ + (joint.velocity[0] * rate), 2 * M_PI);
    positon_joint_right_ = fmod(positon_joint_right_ + (joint.velocity[1] * rate), 2 * M_PI);
    joint.position[0] = positon_joint_left_;
    joint.position[1] = positon_joint_right_;
    joint.effort[0] = k_ele_left * motor_left.current;
    joint.effort[1] = k_ele_right * motor_right.current;
    joint_pub_.publish(joint);
}

void ROSMotionController::sendOdom(serial_bridge::Velocity velocity, serial_bridge::Pose pose) {
    ros::Time current_time = ros::Time::now();
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = tf_odometry_string_;
    odom_trans.child_frame_id = tf_base_link_string_;
    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster_.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = tf_odometry_string_;

    //set the position
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = tf_base_link_string_;
    odom.twist.twist.linear.x = velocity.lin_vel * cos(pose.theta);
    odom.twist.twist.linear.y = velocity.lin_vel * sin(pose.theta);
    odom.twist.twist.angular.z = velocity.ang_vel;

    //publish the message
    odom_pub_.publish(odom);
}