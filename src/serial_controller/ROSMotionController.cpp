/* 
 * File:   ROSMotionController.cpp
 * Author: Raffaello Bonghi
 * 
 * Created on 15 November 2013, 18:34
 */

#include "serial_controller/ROSMotionController.h"

#define NUMBER_PUB 10

using namespace std;

ROSMotionController::ROSMotionController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial)
: ROSController(name_node, nh, serial) {

    string param_name_board = "Motion Control";
    if (name_board.compare(param_name_board) == 0) {
        nh_.setParam(name_node + "/name_board", name_board);
    } else {
        throw (controller_exception("Other board: " + name_board));
    }

    serial->addCallback(&ROSMotionController::motionPacket, this, HASHMAP_MOTION);
    addVectorPacketRequest(&ROSMotionController::updatePacket, this);
    addParameterPacketRequest(&ROSMotionController::addParameter, this);
    addTimerEvent(&ROSMotionController::timerEvent, this);

    //Open Publisher
    //- Command receive
    pub_pose = nh_.advertise<serial_bridge::Pose>(name_node + "/pose", NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_enable = nh_.advertise<serial_bridge::Enable>(name_node + "/enable", NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_motor_left = nh_.advertise<serial_bridge::Motor>(name_node + "/motor/" + left_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_motor_right = nh_.advertise<serial_bridge::Motor>(name_node + "/motor/" + right_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_velocity = nh_.advertise<serial_bridge::Velocity>(name_node + "/velocity", NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_velocity_meas = nh_.advertise<serial_bridge::Velocity>(name_node + "/" + measure_string + "/velocity", NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    //-- Conventional (Using TF, NAV)
    pub_twist = nh_.advertise<geometry_msgs::Twist>(name_node + "/twist", NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_odom = nh_.advertise<nav_msgs::Odometry>(name_node + "/odometry", NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    //JointState position
    pub_joint = nh_.advertise<sensor_msgs::JointState>(joint_string, 1,
            boost::bind(&ROSController::connectCallback, this, _1));

    //Open Subscriber
    //- Command
    sub_velocity = nh_.subscribe(name_node + "/" + command_string + "/velocity", 1, &ROSMotionController::velocityCallback, this);
    sub_pose = nh_.subscribe(name_node + "/" + command_string + "/pose", 1, &ROSMotionController::poseCallback, this);
    sub_enable = nh_.subscribe(name_node + "/" + command_string + "/enable", 1, &ROSMotionController::enableCallback, this);
    //-- Conventional (Using TF, NAV)
    sub_pose_estimate = nh_.subscribe(name_node + "/" + command_string + "/odometry", 1, &ROSMotionController::poseTFCallback, this);
    sub_twist = nh_.subscribe(name_node + "/" + command_string + "/twist", 1, &ROSMotionController::twistCallback, this);

    //Open Service
    srv_pid = nh_.advertiseService(name_node + "/pid", &ROSMotionController::pidServiceCallback, this);
    srv_parameter = nh_.advertiseService(name_node + "/parameter", &ROSMotionController::parameterServiceCallback, this);
    srv_constraint = nh_.advertiseService(name_node + "/constraint", &ROSMotionController::constraintServiceCallback, this);
}

ROSMotionController::~ROSMotionController() {
    serial_->clearCallback(HASHMAP_MOTION);
    clearVectorPacketRequest();
    clearParameterPacketRequest();
    clearTimerEvent();
}

void ROSMotionController::addParameter(std::vector<information_packet_t>* list_send) {
    //Constraint
    if (nh_.hasParam(name_node_ + "/" + joint_string + "/constraint")) {
        ROS_DEBUG("Sync parameter /constraint: ROS -> ROBOT");
        constraint_t constraint = get_constraint();
        list_send->push_back(serial_->createDataPacket(CONSTRAINT, HASHMAP_MOTION, (abstract_packet_t*) & constraint));
    } else {
        ROS_DEBUG("Sync parameter /constraint: ROBOT -> ROS");
        list_send->push_back(serial_->createPacket(CONSTRAINT, REQUEST, HASHMAP_MOTION));
    }
    //Load PID/Left
    if (nh_.hasParam(name_node_ + +"/pid/" + left_string)) {
        ROS_DEBUG("Sync parameter /pid/%s: ROS -> ROBOT", left_string.c_str());
        pid_control_t pid_l = get_pid(left_string);
        list_send->push_back(serial_->createDataPacket(PID_CONTROL_L, HASHMAP_MOTION, (abstract_packet_t*) & pid_l));
    } else {
        ROS_DEBUG("Sync parameter /pid/%s: ROBOT -> ROS", left_string.c_str());
        list_send->push_back(serial_->createPacket(PID_CONTROL_L, REQUEST, HASHMAP_MOTION));
    }
    //Load PID/Right
    if (nh_.hasParam(name_node_ + +"/pid/" + right_string)) {
        ROS_DEBUG("Sync parameter /pid/%s: ROS -> ROBOT", right_string.c_str());
        pid_control_t pid_r = get_pid(right_string);
        list_send->push_back(serial_->createDataPacket(PID_CONTROL_R, HASHMAP_MOTION, (abstract_packet_t*) & pid_r));
    } else {
        ROS_DEBUG("Sync parameter /pid/%s: ROBOT -> ROS", right_string.c_str());
        list_send->push_back(serial_->createPacket(PID_CONTROL_R, REQUEST, HASHMAP_MOTION));
    }
    //Parameter motors
    if (nh_.hasParam(name_node_ + "/structure")) {
        ROS_DEBUG("Sync parameter /structure: ROS -> ROBOT");
        parameter_motors_t parameter_motors = get_parameter();
        list_send->push_back(serial_->createDataPacket(PARAMETER_MOTORS, HASHMAP_MOTION, (abstract_packet_t*) & parameter_motors));
    } else {
        ROS_DEBUG("Sync parameter /structure: ROBOT -> ROS");
        list_send->push_back(serial_->createPacket(PARAMETER_MOTORS, REQUEST, HASHMAP_MOTION));
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
    if (!nh_.hasParam(name_node_ + "/" + joint_string + "/back_emf")) {
        ROS_DEBUG("Sync parameter %s/back_emf: set", joint_string.c_str());
        nh_.setParam("/" + name_node_ + "/" + joint_string + "/back_emf/" + left_string, 1.0);
        nh_.setParam("/" + name_node_ + "/" + joint_string + "/back_emf/" + right_string, 1.0);
    }
    joint.header.frame_id = tf_joint_string_;
    joint.name.resize(2);
    joint.effort.resize(2);
    joint.velocity.resize(2);
    joint.position.resize(2);
    joint.name[0] = left_string;
    joint.name[1] = right_string;
}

void ROSMotionController::motionPacket(const unsigned char& command, const abstract_packet_t* packet) {
    switch (command) {
        case CONSTRAINT:
            nh_.setParam(name_node_ + "/" + joint_string + "/constraint/" + right_string, packet->constraint.max_right);
            nh_.setParam(name_node_ + "/" + joint_string + "/constraint/" + left_string, packet->constraint.max_left);
            break;
        case PARAMETER_MOTORS:
            nh_.setParam(name_node_ + "/structure/" + wheelbase_string, packet->parameter_motors.wheelbase);
            nh_.setParam(name_node_ + "/structure/" + radius_string + "/" + right_string, packet->parameter_motors.radius_r);
            nh_.setParam(name_node_ + "/structure/" + radius_string + "/" + left_string, packet->parameter_motors.radius_l);
            nh_.setParam(name_node_ + "/" + joint_string + "/k_vel/" + right_string, packet->parameter_motors.k_vel_r);
            nh_.setParam(name_node_ + "/" + joint_string + "/k_vel/" + left_string, packet->parameter_motors.k_vel_l);
            nh_.setParam(name_node_ + "/" + joint_string + "/k_ang/" + right_string, packet->parameter_motors.k_ang_r);
            nh_.setParam(name_node_ + "/" + joint_string + "/k_ang/" + left_string, packet->parameter_motors.k_ang_l);
            nh_.setParam(name_node_ + "/" + joint_string + "/pwm_bit", packet->parameter_motors.pwm_step);
            nh_.setParam(name_node_ + "/odo_mis_step", packet->parameter_motors.sp_min);
            pwm_motor = packet->parameter_motors.pwm_step;
            break;
        case PID_CONTROL_L:
            name_pid = name_node_ + "/pid/" + left_string + "/";
            nh_.setParam(name_pid + "P", packet->pid.kp);
            nh_.setParam(name_pid + "I", packet->pid.ki);
            nh_.setParam(name_pid + "D", packet->pid.kd);
            break;
        case PID_CONTROL_R:
            name_pid = "/" + name_node_ + "/pid/" + right_string + "/";
            nh_.setParam(name_pid + "P", packet->pid.kp);
            nh_.setParam(name_pid + "I", packet->pid.ki);
            nh_.setParam(name_pid + "D", packet->pid.kd);
            break;
        case MOTOR_L:
            motor_left.reference = ((double) packet->motor.rifer_vel) / 1000;
            motor_left.control = -((double) packet->motor.control_vel - pwm_motor / 2) / (pwm_motor / 2);
            motor_left.measure = ((double) packet->motor.measure_vel) / 1000;
            motor_left.current = ((double) packet->motor.current) / 1000;
            pub_motor_left.publish(motor_left);
            break;
        case MOTOR_R:
            motor_right.reference = ((double) packet->motor.rifer_vel) / 1000;
            motor_right.control = ((double) packet->motor.control_vel - pwm_motor / 2) / (pwm_motor / 2);
            motor_right.measure = ((double) packet->motor.measure_vel) / 1000;
            motor_right.current = ((double) packet->motor.current) / 1000;
            pub_motor_right.publish(motor_right);
            break;
        case COORDINATE:
            pose.x = packet->coordinate.x;
            pose.y = packet->coordinate.y;
            pose.theta = packet->coordinate.theta;
            pose.space = packet->coordinate.space;
            pub_pose.publish(pose);
            break;
        case VELOCITY:
            cmd_velocity.linear = packet->velocity.v;
            cmd_velocity.angular = packet->velocity.w;
            pub_velocity.publish(cmd_velocity);
            twist.linear.x = packet->velocity.v;
            twist.angular.z = packet->velocity.w;
            pub_twist.publish(twist);
            break;
        case VELOCITY_MIS:
            meas_velocity.linear = packet->velocity.v;
            meas_velocity.angular = packet->velocity.w;
            pub_velocity_meas.publish(meas_velocity);
            break;
        case ENABLE:
            enable_motors.enable = packet->enable;
            pub_enable.publish(enable_motors);
            break;
    }
}

void ROSMotionController::timerEvent(const ros::TimerEvent& event) {
    // Send Odometry message
    if (pub_odom.getNumSubscribers() >= 1) {
        sendOdometry(&meas_velocity, &pose);
    }
    // Send JointState message
    if (pub_joint.getNumSubscribers() >= 1) {
        sendJointState(&motor_left, &motor_right);
    }
}

void ROSMotionController::updatePacket(std::vector<information_packet_t>* list_send) {
    std::string packet_string;
    if ((pub_pose.getNumSubscribers() >= 1) || (pub_odom.getNumSubscribers() >= 1)) {
        packet_string += "Odo ";
        list_send->push_back(serial_->createPacket(COORDINATE, REQUEST, HASHMAP_MOTION));
    }
    if ((pub_velocity.getNumSubscribers() >= 1) || (pub_twist.getNumSubscribers() >= 1)) {
        packet_string += "Vel ";
        list_send->push_back(serial_->createPacket(VELOCITY, REQUEST, HASHMAP_MOTION));
    }
    if (pub_enable.getNumSubscribers() >= 1) {
        packet_string += "Ena ";
        list_send->push_back(serial_->createPacket(ENABLE, REQUEST, HASHMAP_MOTION));
    }
    if ((pub_velocity_meas.getNumSubscribers() >= 1) || (pub_odom.getNumSubscribers() >= 1)) {
        packet_string += "VelMis ";
        list_send->push_back(serial_->createPacket(VELOCITY_MIS, REQUEST, HASHMAP_MOTION));
    }
    if ((pub_motor_left.getNumSubscribers() >= 1) || (pub_joint.getNumSubscribers() >= 1)) {
        packet_string += "MotL ";
        list_send->push_back(serial_->createPacket(MOTOR_L, REQUEST, HASHMAP_MOTION));
    }
    if ((pub_motor_right.getNumSubscribers() >= 1) || (pub_joint.getNumSubscribers() >= 1)) {
        packet_string += "MotR ";
        list_send->push_back(serial_->createPacket(MOTOR_R, REQUEST, HASHMAP_MOTION));
    }
    //    ROS_INFO("[ %s]", packet_string.c_str());
}

void ROSMotionController::velocityCallback(const serial_bridge::Velocity::ConstPtr &msg) {
    velocity_t velocity;
    velocity.v = msg->linear;
    velocity.w = msg->angular;
    try {
        serial_->parserSendPacket(serial_->createDataPacket(VELOCITY, HASHMAP_MOTION, (abstract_packet_t*) & velocity), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void ROSMotionController::twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    velocity_t velocity;
    velocity.v = msg->linear.x;
    velocity.w = msg->angular.z;
    try {
        serial_->parserSendPacket(serial_->createDataPacket(VELOCITY, HASHMAP_MOTION, (abstract_packet_t*) & velocity), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void ROSMotionController::enableCallback(const serial_bridge::Enable::ConstPtr &msg) {
    enable_motor_t enable = msg->enable;
    try {
        serial_->parserSendPacket(serial_->createDataPacket(ENABLE, HASHMAP_MOTION, (abstract_packet_t*) & enable), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void ROSMotionController::poseCallback(const serial_bridge::Pose::ConstPtr &msg) {
    saveOdometry(msg.get());
}

void ROSMotionController::poseTFCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    serial_bridge::Pose pose;
    pose.x = msg.get()->pose.pose.position.x;
    pose.y = msg.get()->pose.pose.position.y;
    pose.theta = tf::getYaw(msg.get()->pose.pose.orientation);
    pose.space = pose.space;
    saveOdometry(&pose);
}

void ROSMotionController::saveOdometry(const serial_bridge::Pose* pose) {
    coordinate_t coordinate;
    coordinate.x = pose->x;
    coordinate.y = pose->y;
    coordinate.theta = pose->theta;
    coordinate.space = pose->space;
    try {
        serial_->parserSendPacket(serial_->createDataPacket(COORDINATE, HASHMAP_MOTION, (abstract_packet_t*) & coordinate), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void ROSMotionController::sendOdometry(const serial_bridge::Velocity* velocity, const serial_bridge::Pose* pose) {
    ros::Time current_time = ros::Time::now();
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose->theta);

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = tf_odometry_string_;
    odom_trans.child_frame_id = tf_base_link_string_;
    odom_trans.transform.translation.x = pose->x;
    odom_trans.transform.translation.y = pose->y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = tf_odometry_string_;

    //set the position
    odom.pose.pose.position.x = pose->x;
    odom.pose.pose.position.y = pose->y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = tf_base_link_string_;
    odom.twist.twist.linear.x = velocity->linear;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = velocity->angular;

    //publish the message
    pub_odom.publish(odom);
}

void ROSMotionController::sendJointState(serial_bridge::Motor* motor_left, serial_bridge::Motor* motor_right) {
    ros::Time now = ros::Time::now();
    double rate = (now - old_time).toSec();
    old_time = now;
    nh_.getParam(name_node_ + "/" + joint_string + "/back_emf/" + left_string, k_ele_left);
    nh_.getParam(name_node_ + "/" + joint_string + "/back_emf/" + right_string, k_ele_right);

    joint.header.stamp = now;
    joint.velocity[0] = motor_left->measure;
    joint.velocity[1] = motor_right->measure;
    positon_joint_left = fmod(positon_joint_left + (joint.velocity[0] * rate), 2 * M_PI);
    positon_joint_right = fmod(positon_joint_right + (joint.velocity[1] * rate), 2 * M_PI);
    joint.position[0] = positon_joint_left;
    joint.position[1] = positon_joint_right;
    joint.effort[0] = k_ele_left * motor_left->current;
    joint.effort[1] = k_ele_right * motor_right->current;
    pub_joint.publish(joint);
}

pid_control_t ROSMotionController::get_pid(std::string name) {
    pid_control_t pid;
    double temp;
    std::string name_pid = name_node_ + "/pid/" + name + "/";
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

    nh_.getParam(name_node_ + "/structure/" + wheelbase_string, temp);
    parameter.wheelbase = temp;
    nh_.getParam(name_node_ + "/structure/" + radius_string + "/" + right_string, temp);
    parameter.radius_r = temp;
    nh_.getParam(name_node_ + "/structure/" + radius_string + "/" + left_string, temp);
    parameter.radius_l = temp;
    nh_.getParam(name_node_ + "/" + joint_string + "/k_vel/" + right_string, temp);
    parameter.k_vel_r = temp;
    nh_.getParam(name_node_ + "/" + joint_string + "/k_vel/" + left_string, temp);
    parameter.k_vel_l = temp;
    nh_.getParam(name_node_ + "/" + joint_string + "/k_ang/" + right_string, temp);
    parameter.k_ang_r = temp;
    nh_.getParam(name_node_ + "/" + joint_string + "/k_ang/" + left_string, temp);
    parameter.k_ang_l = temp;
    nh_.getParam(name_node_ + "/odo_mis_step", temp);
    parameter.sp_min = temp;
    nh_.getParam(name_node_ + "/" + joint_string + "/pwm_bit", temp_int);
    parameter.pwm_step = temp_int;
    return parameter;
}

constraint_t ROSMotionController::get_constraint() {
    constraint_t constraint;
    double temp;

    nh_.getParam(name_node_ + "/" + joint_string + "/constraint/" + right_string, temp);
    constraint.max_right = temp;
    nh_.getParam(name_node_ + "/" + joint_string + "/constraint/" + left_string, temp);
    constraint.max_left = temp;
    return constraint;
}

bool ROSMotionController::pidServiceCallback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&) {
    std::string name = req.name;
    pid_control_t pid;
    std::vector<information_packet_t> list_send;
    ROS_INFO("PID UPDATE");
    if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0)) {
        pid = get_pid(left_string);
        list_send.push_back(serial_->createDataPacket(PID_CONTROL_L, HASHMAP_MOTION, (abstract_packet_t*) & pid));
    }
    if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0)) {
        pid = get_pid(right_string);
        list_send.push_back(serial_->createDataPacket(PID_CONTROL_R, HASHMAP_MOTION, (abstract_packet_t*) & pid));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}

bool ROSMotionController::parameterServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    parameter_motors_t parameter = get_parameter();
    try {
        serial_->parserSendPacket(serial_->createDataPacket(PARAMETER_MOTORS, HASHMAP_MOTION, (abstract_packet_t*) & parameter), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}

bool ROSMotionController::constraintServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    constraint_t constraint = get_constraint();
    try {
        serial_->parserSendPacket(serial_->createDataPacket(CONSTRAINT, HASHMAP_MOTION, (abstract_packet_t*) & constraint), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}