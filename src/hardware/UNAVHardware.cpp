/* 
 * File:   ROSMotionController.cpp
 * Author: Raffaello Bonghi
 * 
 * Created on 15 November 2013, 18:34
 */

#include "hardware/UNAVHardware.h"
#include <limits>

#include <boost/assign/list_of.hpp>

#define NUMBER_PUB 10
#define SGN(x)  ( ((x) < 0) ?  -1 : ( ((x) == 0 ) ? 0 : 1) )

using namespace std;

UNAVHardware::UNAVHardware(const ros::NodeHandle& nh, ParserPacket* serial)
: ORBHardware(nh, serial), positon_joint_left(0), positon_joint_right(0) {

    string param_type_board = "Motor Control";
    if (type_board.compare(param_type_board) == 0) {
        nh_.setParam("info/type_board", type_board);
    } else {
        throw (controller_exception("Other board: " + type_board));
    }

    serial->addCallback(&UNAVHardware::motionPacket, this, HASHMAP_MOTION);
    addVectorPacketRequest(&UNAVHardware::updatePacket, this);
    addParameterPacketRequest(&UNAVHardware::addParameter, this);
    addTimerEvent(&UNAVHardware::timerEvent, this);
    addAliveOperation(&UNAVHardware::aliveOperation, this);

    //Open Service
    srv_pid = nh_.advertiseService("pid", &UNAVHardware::pidServiceCallback, this);
    srv_parameter = nh_.advertiseService("parameter", &UNAVHardware::parameterServiceCallback, this);
    srv_constraint = nh_.advertiseService("constraint", &UNAVHardware::constraintServiceCallback, this);
    srv_emergency = nh_.advertiseService("emergency", &UNAVHardware::emergencyServiceCallback, this);

    registerControlInterfaces();

    status[0] = STATE_CONTROL_DISABLE;
    status[1] = STATE_CONTROL_DISABLE;
}

UNAVHardware::~UNAVHardware() {
    serial_->clearCallback(HASHMAP_MOTION);
    clearVectorPacketRequest();
    clearParameterPacketRequest();
    clearTimerEvent();
    clearAliveOperation();
}

void UNAVHardware::addParameter(std::vector<information_packet_t>* list_send) {
    //Constraint
    ROS_INFO("%s/constraint",joint_string.c_str());
    if (nh_.hasParam(joint_string + "/constraint")) {
        ROS_INFO("Sync parameter constraint: ROS -> ROBOT");
        constraint_t constraint = get_constraint();
        list_send->push_back(serial_->createDataPacket(CONSTRAINT, HASHMAP_MOTION, (abstract_message_u*) & constraint));
    } else {
        ROS_INFO("Sync parameter constraint: ROBOT -> ROS");
        list_send->push_back(serial_->createPacket(CONSTRAINT, REQUEST, HASHMAP_MOTION));
    }
    //Load PID/Left
    if (nh_.hasParam("pid/" + left_string)) {
        ROS_INFO("Sync parameter pid/%s: ROS -> ROBOT", left_string.c_str());
        pid_control_t pid_l = get_pid(left_string);
        list_send->push_back(serial_->createDataPacket(PID_CONTROL_L, HASHMAP_MOTION, (abstract_message_u*) & pid_l));
    } else {
        ROS_INFO("Sync parameter pid/%s: ROBOT -> ROS", left_string.c_str());
        list_send->push_back(serial_->createPacket(PID_CONTROL_L, REQUEST, HASHMAP_MOTION));
    }
    //Load PID/Right
    if (nh_.hasParam("pid/" + right_string)) {
        ROS_INFO("Sync parameter pid/%s: ROS -> ROBOT", right_string.c_str());
        pid_control_t pid_r = get_pid(right_string);
        list_send->push_back(serial_->createDataPacket(PID_CONTROL_R, HASHMAP_MOTION, (abstract_message_u*) & pid_r));
    } else {
        ROS_INFO("Sync parameter pid/%s: ROBOT -> ROS", right_string.c_str());
        list_send->push_back(serial_->createPacket(PID_CONTROL_R, REQUEST, HASHMAP_MOTION));
    }
    //Parameter unicycle
    if (nh_.hasParam("structure")) {
        ROS_INFO("Sync parameter structure: ROS -> ROBOT");
        parameter_unicycle_t parameter_unicycle = get_unicycle_parameter();
        list_send->push_back(serial_->createDataPacket(PARAMETER_UNICYCLE, HASHMAP_MOTION, (abstract_message_u*) & parameter_unicycle));
    } else {
        ROS_INFO("Sync parameter structure: ROBOT -> ROS");
        list_send->push_back(serial_->createPacket(PARAMETER_UNICYCLE, REQUEST, HASHMAP_MOTION));
    }
    //Parameter motors LEFT
    if (nh_.hasParam(joint_string + "/" + left_string)) {
        ROS_INFO("Sync parameter parameter motor/%s: ROS -> ROBOT",left_string.c_str());
        parameter_motor_t parameter_motor = get_motor_parameter(left_string);
        list_send->push_back(serial_->createDataPacket(PARAMETER_MOTOR_L, HASHMAP_MOTION, (abstract_message_u*) & parameter_motor));
    } else {
        ROS_INFO("Sync parameter parameter motor/%s: ROBOT -> ROS",left_string.c_str());
        list_send->push_back(serial_->createPacket(PARAMETER_MOTOR_L, REQUEST, HASHMAP_MOTION));
    }
    //Parameter motors RIGHT
    if (nh_.hasParam(joint_string + "/" + right_string)) {
        ROS_INFO("Sync parameter parameter motor/%s: ROS -> ROBOT",right_string.c_str());
        parameter_motor_t parameter_motor = get_motor_parameter(right_string);
        list_send->push_back(serial_->createDataPacket(PARAMETER_MOTOR_R, HASHMAP_MOTION, (abstract_message_u*) & parameter_motor));
    } else {
        ROS_INFO("Sync parameter parameter motor/%s: ROBOT -> ROS",right_string.c_str());
        list_send->push_back(serial_->createPacket(PARAMETER_MOTOR_R, REQUEST, HASHMAP_MOTION));
    }
    //Names TF
    if (nh_.hasParam(tf_string)) {
        ROS_INFO("Sync parameter %s: load", tf_string.c_str());
        nh_.param<std::string>(tf_string + "/" + odometry_string, tf_odometry_string_, tf_odometry_string_);
        nh_.param<std::string>(tf_string + "/" + base_link_string, tf_base_link_string_, tf_base_link_string_);
        nh_.param<std::string>(tf_string + "/" + joint_string, tf_joint_string_, tf_joint_string_);
    } else {
        ROS_INFO("Sync parameter %s: set", tf_string.c_str());
        tf_odometry_string_ = odometry_string;
        tf_base_link_string_ = base_link_string;
        tf_joint_string_ = joint_string;
        nh_.setParam(tf_string + "/" + odometry_string, tf_odometry_string_);
        nh_.setParam(tf_string + "/" + base_link_string, tf_base_link_string_);
        nh_.setParam(tf_string + "/" + joint_string, tf_joint_string_);
    }
    //Names ele
    if (!nh_.hasParam(joint_string + "/back_emf")) {
        ROS_INFO("Sync parameter %s/back_emf: set", joint_string.c_str());
        nh_.setParam(joint_string + "/back_emf/" + left_string, 1.0);
        nh_.setParam(joint_string + "/back_emf/" + right_string, 1.0);
    }
    //Set timer rate
    if (nh_.hasParam(emergency_string)) {
        ROS_INFO("Sync parameter %s: ROS -> ROBOT", emergency_string.c_str());
        emergency_t emergency = get_emergency();
        list_send->push_back(serial_->createDataPacket(EMERGENCY, HASHMAP_MOTION, (abstract_message_u*) & emergency));
    } else {
        ROS_INFO("Sync parameter %s: ROBOT -> ROS", emergency_string.c_str());
        list_send->push_back(serial_->createPacket(EMERGENCY, REQUEST, HASHMAP_MOTION));
    }
    joint.header.frame_id = tf_joint_string_;
    joint.name.resize(2);
    joint.effort.resize(2);
    joint.velocity.resize(2);
    joint.position.resize(2);
    joint.name[0] = left_string;
    joint.name[1] = right_string;
}

void UNAVHardware::motionPacket(const unsigned char& command, const abstract_message_u* packet) {
    switch (command) {
        case CONSTRAINT:
            nh_.setParam(joint_string + "/constraint/" + right_string, packet->constraint.max_right);
            nh_.setParam(joint_string + "/constraint/" + left_string, packet->constraint.max_left);
            break;
        case PARAMETER_UNICYCLE:
            nh_.setParam("structure/" + wheelbase_string, packet->parameter_unicycle.wheelbase);
            nh_.setParam("structure/" + radius_string + "/" + right_string, packet->parameter_unicycle.radius_r);
            nh_.setParam("structure/" + radius_string + "/" + left_string, packet->parameter_unicycle.radius_l);
            nh_.setParam("odo_mis_step", packet->parameter_unicycle.sp_min);
            break;
        case PARAMETER_MOTOR_L:
            nh_.setParam(joint_string + "/" + left_string + "/k_vel", packet->parameter_motor.k_vel);
            nh_.setParam(joint_string + "/" + left_string + "/k_ang", packet->parameter_motor.k_ang);
            nh_.setParam(joint_string + "/" + left_string + "/encoder_swap", packet->parameter_motor.versus);
            nh_.setParam(joint_string + "/" + left_string + "/default_enable", packet->parameter_motor.enable_set);
            break;
        case PARAMETER_MOTOR_R:
            nh_.setParam(joint_string + "/" + right_string + "/k_vel", packet->parameter_motor.k_vel);
            nh_.setParam(joint_string + "/" + right_string + "/k_ang", packet->parameter_motor.k_ang);
            nh_.setParam(joint_string + "/" + right_string + "/versus", packet->parameter_motor.versus);
            nh_.setParam(joint_string + "/" + right_string + "/default_enable", packet->parameter_motor.enable_set);
            break;
        case EMERGENCY:
            nh_.setParam(emergency_string + "/bridge_off", packet->emergency.bridge_off);
            nh_.setParam(emergency_string + "/slope_time", packet->emergency.slope_time);
            nh_.setParam(emergency_string + "/timeout", ((double) packet->emergency.timeout) / 1000.0);
        break;
        case PID_CONTROL_L:
            name_pid = "pid/" + left_string + "/";
            nh_.setParam(name_pid + "P", packet->pid.kp);
            nh_.setParam(name_pid + "I", packet->pid.ki);
            nh_.setParam(name_pid + "D", packet->pid.kd);
            break;
        case PID_CONTROL_R:
            name_pid = "pid/" + right_string + "/";
            nh_.setParam(name_pid + "P", packet->pid.kp);
            nh_.setParam(name_pid + "I", packet->pid.ki);
            nh_.setParam(name_pid + "D", packet->pid.kd);
            break;
        case VEL_MOTOR_MIS_L:

            break;
        case VEL_MOTOR_MIS_R:

            break;
        case MOTOR_L:
            motor_left.reference = ((double) packet->motor.refer_vel) / 1000;
            motor_left.control = ((double) packet->motor.control_vel) * (1000.0 / INT16_MAX);
            motor_left.measure = ((double) packet->motor.measure_vel) / 1000;
            motor_left.current = ((double) packet->motor.current) / 1000;
            //pub_motor_left.publish(motor_left);
            break;
        case MOTOR_R:
            motor_right.reference = ((double) packet->motor.refer_vel) / 1000;
            motor_right.control = ((double) packet->motor.control_vel) * (1000.0 / INT16_MAX);
            motor_right.measure = ((double) packet->motor.measure_vel) / 1000;
            motor_right.current = ((double) packet->motor.current) / 1000;
            //pub_motor_right.publish(motor_right);
            break;
        case COORDINATE:
            pose.x = packet->coordinate.x;
            pose.y = packet->coordinate.y;
            pose.theta = packet->coordinate.theta;
            pose.space = packet->coordinate.space;
            //pub_pose.publish(pose);
            break;
        case VELOCITY:
            twist.linear.x = packet->velocity.v;
            twist.angular.z = packet->velocity.w;
            //pub_twist.publish(twist);
            break;
        case VELOCITY_MIS:
            meas_velocity = packet->velocity;
            break;
        case ENABLE:
            enable_motors.enable = packet->enable;
            //pub_enable.publish(enable_motors);
            break;
    }
}

void UNAVHardware::registerControlInterfaces() {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
        ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
          joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
}

void UNAVHardware::updateJointsFromHardware() {

}

void UNAVHardware::writeCommandsToHardware() {

}

void UNAVHardware::timerEvent(const ros::TimerEvent& event) {
//    // Send Odometry message
//    if (pub_odom.getNumSubscribers() >= 1) {
//        sendOdometry(&meas_velocity, &pose);
//    }
//    // Send JointState message
//    if (pub_joint.getNumSubscribers() >= 1) {
//        sendJointState(&motor_left, &motor_right);
//    }
}

bool UNAVHardware::aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_send) {
//    if (sub_twist.getNumPublishers() >= 1) {
//        list_send->push_back(serial_->createDataPacket(VEL_MOTOR_L, HASHMAP_MOTION, (abstract_message_u*) & velocity_ref[0]));
//        list_send->push_back(serial_->createDataPacket(VEL_MOTOR_R, HASHMAP_MOTION, (abstract_message_u*) & velocity_ref[1]));
//        return true;
//    } else {
//        if (status[0] != STATE_CONTROL_EMERGENCY || status[1] != STATE_CONTROL_EMERGENCY) {
//            status[0] = STATE_CONTROL_EMERGENCY;
//            status[1] = STATE_CONTROL_EMERGENCY;
//            list_send->push_back(serial_->createDataPacket(ENABLE_MOTOR_L, HASHMAP_MOTION, (abstract_message_u*) & status[0]));
//            list_send->push_back(serial_->createDataPacket(ENABLE_MOTOR_R, HASHMAP_MOTION, (abstract_message_u*) & status[1]));
//            return true;
//        } else {
//            return false;
//        }
//    }
}

void UNAVHardware::updatePacket(std::vector<information_packet_t>* list_send) {
    std::string packet_string;
//    if ((pub_pose.getNumSubscribers() >= 1) || (pub_odom.getNumSubscribers() >= 1)) {
//        packet_string += "Odo ";
//        list_send->push_back(serial_->createPacket(COORDINATE, REQUEST, HASHMAP_MOTION));
//    }
//    if (pub_twist.getNumSubscribers() >= 1) {
//        packet_string += "Vel ";
//        list_send->push_back(serial_->createPacket(VELOCITY, REQUEST, HASHMAP_MOTION));
//    }
//    if (pub_enable.getNumSubscribers() >= 1) {
//        packet_string += "Ena ";
//        list_send->push_back(serial_->createPacket(ENABLE, REQUEST, HASHMAP_MOTION));
//    }
//    if (pub_odom.getNumSubscribers() >= 1) {
//        packet_string += "VelMis ";
//        list_send->push_back(serial_->createPacket(VELOCITY_MIS, REQUEST, HASHMAP_MOTION));
//    }
//    if ((pub_motor_left.getNumSubscribers() >= 1) || (pub_joint.getNumSubscribers() >= 1)) {
//        packet_string += "MotL ";
//        list_send->push_back(serial_->createPacket(MOTOR_L, REQUEST, HASHMAP_MOTION));
//    }
//    if ((pub_motor_right.getNumSubscribers() >= 1) || (pub_joint.getNumSubscribers() >= 1)) {
//        packet_string += "MotR ";
//        list_send->push_back(serial_->createPacket(MOTOR_R, REQUEST, HASHMAP_MOTION));
//    }
    //    ROS_INFO("[ %s]", packet_string.c_str());
}

void UNAVHardware::ConverToMotorVelocity(const geometry_msgs::Twist* msg, motor_control_t *motor_ref) {
    parameter_unicycle_t parameter_unicycle = get_unicycle_parameter();
    // wl = 1/rl [ 1, -d/2]
    // wr = 1/rr [ 1,  d/2]
    long int motor_ref_long[2];
    //Left motor
    motor_ref_long[0] = (long int) ((1.0f / parameter_unicycle.radius_l)*(msg->linear.x - (0.5f*parameter_unicycle.wheelbase * (msg->angular.z)))*1000);
    //Right Motor
    motor_ref_long[1] = (long int) ((1.0f / parameter_unicycle.radius_r)*(msg->linear.x + (0.5f*parameter_unicycle.wheelbase * (msg->angular.z)))*1000);

    // >>>>> Saturation on 16 bit values
    if(motor_ref_long[0] > 32767) {
        motor_ref[0] = 32767;
    } else if (motor_ref_long[0] < -32768) {
        motor_ref_long[0] = -32768;
    } else {
        motor_ref[0] = (int16_t) motor_ref_long[0];
    }

    if(motor_ref_long[1] > 32767) {
        motor_ref[1] = 32767;
    } else if (motor_ref_long[1] < -32768) {
        motor_ref_long[1] = -32768;
    } else {
        motor_ref[1] = (int16_t) motor_ref_long[1];
    }
    // <<<<< Saturation on 16 bit values

}

pid_control_t UNAVHardware::get_pid(std::string name) {
    pid_control_t pid;
    double temp;
    std::string name_pid = "pid/" + name + "/";
    nh_.getParam(name_pid + "P", temp);
    pid.kp = temp;
    nh_.getParam(name_pid + "I", temp);
    pid.ki = temp;
    nh_.getParam(name_pid + "D", temp);
    pid.kd = temp;
    return pid;
}

parameter_motor_t UNAVHardware::get_motor_parameter(std::string name) {
    parameter_motor_t parameter;
    double temp;
    int temp2;

    nh_.getParam(joint_string + "/" + name + "/k_vel", temp);
    parameter.k_vel = temp;
    nh_.getParam(joint_string + "/" + name + "/k_ang", temp);
    parameter.k_ang = temp;
    nh_.getParam(joint_string + "/" + name + "/versus", temp2);
    parameter.versus = temp2;
    nh_.getParam(joint_string + "/" + name + "/default_enable", temp2);
    parameter.enable_set = temp2;
    return parameter;
}

parameter_unicycle_t UNAVHardware::get_unicycle_parameter() {
    parameter_unicycle_t parameter;
    double temp;

    nh_.getParam("structure/" + wheelbase_string, temp);
    parameter.wheelbase = temp;
    nh_.getParam("structure/" + radius_string + "/" + right_string, temp);
    parameter.radius_r = temp;
    nh_.getParam("structure/" + radius_string + "/" + left_string, temp);
    parameter.radius_l = temp;
    nh_.getParam("odo_mis_step", temp);
    parameter.sp_min = temp;
    return parameter;
}

emergency_t UNAVHardware::get_emergency() {
    emergency_t emergency;
    double temp;
    nh_.getParam(emergency_string + "/bridge_off", temp);
    emergency.bridge_off = temp;
    nh_.getParam(emergency_string + "/slope_time", temp);
    emergency.slope_time = temp;
    nh_.getParam(emergency_string + "/timeout", temp);
    emergency.timeout = (int16_t) (temp * 1000);
    return emergency;
}

constraint_t UNAVHardware::get_constraint() {
    constraint_t constraint;
    int temp;

    nh_.getParam(joint_string + "/constraint/" + right_string, temp);
    constraint.max_right = (int16_t) temp;
    nh_.getParam(joint_string + "/constraint/" + left_string, temp);
    constraint.max_left = (int16_t) temp;
    return constraint;
}

bool UNAVHardware::pidServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
    std::string name = req.name;
    pid_control_t pid;
    std::vector<information_packet_t> list_send;
    ROS_INFO("PID UPDATE");
    if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0)) {
        pid = get_pid(left_string);
        list_send.push_back(serial_->createDataPacket(PID_CONTROL_L, HASHMAP_MOTION, (abstract_message_u*) & pid));
    }
    if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0)) {
        pid = get_pid(right_string);
        list_send.push_back(serial_->createDataPacket(PID_CONTROL_R, HASHMAP_MOTION, (abstract_message_u*) & pid));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}

bool UNAVHardware::parameterServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
    std::string name = req.name;
    parameter_motor_t parameter_motor;
    std::vector<information_packet_t> list_send;
    ROS_INFO("PARAMETER UPDATE");
    if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0)) {
        parameter_motor = get_motor_parameter(left_string);
        list_send.push_back(serial_->createDataPacket(PARAMETER_MOTOR_L, HASHMAP_MOTION, (abstract_message_u*) & parameter_motor));
    }
    if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0)) {
        parameter_motor = get_motor_parameter(right_string);
        list_send.push_back(serial_->createDataPacket(PARAMETER_MOTOR_R, HASHMAP_MOTION, (abstract_message_u*) & parameter_motor));
    }
    if ((name.compare(paramenter_unicycle_string) == 0) || (name.compare(all_string) == 0)) {
        parameter_unicycle_t parameter_unicycle = get_unicycle_parameter();
        list_send.push_back(serial_->createDataPacket(PARAMETER_UNICYCLE, HASHMAP_MOTION, (abstract_message_u*) & parameter_unicycle));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}

bool UNAVHardware::constraintServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    constraint_t constraint = get_constraint();
    try {
        serial_->parserSendPacket(serial_->createDataPacket(CONSTRAINT, HASHMAP_MOTION, (abstract_message_u*) & constraint), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}

bool UNAVHardware::emergencyServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    emergency_t emergency = get_emergency();
    try {
        serial_->parserSendPacket(serial_->createDataPacket(EMERGENCY, HASHMAP_MOTION, (abstract_message_u*) & emergency), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}
