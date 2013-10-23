/* 
 * File:   RosControllerSerial.cpp
 * Author: Raffaello Bonghi
 * 
 * Created on June 7, 2013, 4:33 PM
 */

#include "serial_controller/ROSControllerSerial.h"

/*
 *
 */
RosControllerSerial::RosControllerSerial(std::string name_node, const ros::NodeHandle& nh, Serial* serial, int rate)
  : nh_(nh), loop_rate_(rate)
{
  name_node_ = name_node; // Initialize node name
  this->serial_ = serial; // Initialize serial port
  ROS_INFO("Init");
  serial_->asyncPacket(&RosControllerSerial::actionAsync, this);
  rate_ = rate; // Initialize rate


  //Open Publisher
  //- Measure
  velocity_mis_pub_ = nh_.advertise<serial_bridge::Velocity>("/" + name_node + "/" + measure_string + "/" + velocity_string, 1000,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));
  //- Command receive
  pose_pub_ = nh_.advertise<serial_bridge::Pose>("/" + name_node + "/" + pose_string, 1000,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));
  velocity_pub_ = nh_.advertise<serial_bridge::Velocity>("/" + name_node + "/" + velocity_string, 1000,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));
  enable_pub_ = nh_.advertise<serial_bridge::Enable>("/" + name_node + "/" + enable_motors, 1000,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));
  motor_left_pub_ = nh_.advertise<serial_bridge::Motor>("/" + name_node + "/" + motor + "/" + left_string, 1000,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));
  motor_right_pub_ = nh_.advertise<serial_bridge::Motor>("/" + name_node + "/" + motor + "/" + right_string, 1000,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));
  time_process_pub_ = nh_.advertise<serial_bridge::Process>("/" + name_node + "/" + process, 1000,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));
  //-- Conventional (Using TF, NAV)
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/" + name_node + "/" + odometry_string, 1000,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));
  //JointState position
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_string, 1,
    boost::bind(&RosControllerSerial::connectCallback, this, _1));

  //Open Subscriber
  //- Command
  pose_sub_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + pose_string, 1, &RosControllerSerial::poseCallback, this);
  velocity_sub_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + velocity_string, 1, &RosControllerSerial::velocityCallback, this);
  enable_sub_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + enable_motors, 1, &RosControllerSerial::enableCallback, this);
  //-- Conventional (Using TF, NAV)
  pose_estimate_sub_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + odometry_string, 1, &RosControllerSerial::pose_tf_Callback, this);

  //Open Service
  pid_update_srv_ = nh_.advertiseService("/" + name_node + "/" + update_pid_string, &RosControllerSerial::pid_update_Callback, this);
  parameter_update_srv_ = nh_.advertiseService("/" + name_node + "/" + update_parameter_string, &RosControllerSerial::parameter_update_Callback, this);
  constraint_update_srv_ = nh_.advertiseService("/" + name_node + "/" + update_constraint_string, &RosControllerSerial::constraint_update_Callback, this);
  process_update_srv_ = nh_.advertiseService("/" + name_node + "/" + update_process_string, &RosControllerSerial::process_update_Callback, this);
  convert_velocity_srv_ = nh_.advertiseService("/" + name_node + "/" + convert_string, &RosControllerSerial::convert_Callback, this);
  reset_srv_ = nh_.advertiseService("/" + name_node + "/" + service_string, &RosControllerSerial::service_Callback, this);

  //Initialize boolean for encapsulation streaming
  pose_active_ = false;
  enable_active_ = false;
  velocity_active_ = false;
  velocity_mis_active_ = false;
  motor_left_active_ = false;
  motor_right_active_ = false;
  time_process_active_ = false;
  odom_active_ = false;
  joint_active_ = false;
  stream_exit_ = false; //stop streaming
  user_ = false; //More topic connected

  //Initialize length null for streaming packet
  packet_.length = 0;

  //Initialize string motion
  string_process_motion[PROCESS_PID_LEFT] = PID_LEFT_STRING;
  string_process_motion[PROCESS_PID_RIGHT] = PID_RIGHT_STRING;
  string_process_motion[PROCESS_VELOCITY] = VELOCITY_STRING;
  string_process_motion[PROCESS_ODOMETRY] = ODOMETRY_STRING;

  positon_joint_left_ = 0;
  positon_joint_right_ = 0;
}

void RosControllerSerial::actionAsync(packet_t packet)
{
  std::list<information_packet_t> decode_packet = Serial::parsing(NULL, packet);
  for (std::list<information_packet_t>::iterator list_iter = decode_packet.begin(); list_iter != decode_packet.end(); list_iter++)
  {
    information_packet_t packet = (*list_iter);
    if (packet.option == CHANGE)
    {
      switch (packet.command)
      {
        case SERVICES:
          if (packet.packet.services.command == VERSION_CODE)
          {
            char* buff = (char*) packet.packet.services.buffer;
            std::string version(buff, SERVICE_BUFF);
            ROS_INFO("Version code: %s", version.c_str());
          }
          break;
      }
    }
  }
}

RosControllerSerial::~RosControllerSerial()
{
}

boost::thread * RosControllerSerial::run()
{
  thr_ = new boost::thread(boost::bind(&RosControllerSerial::th_stream, this));
  return thr_;
}

void RosControllerSerial::setPacketStream(packet_t packet)
{
  this->packet_ = packet;
}

bool RosControllerSerial::stream_bool()
{
  boost::unique_lock<boost::mutex> lock(mutex_);

  velocity_active_ = !(velocity_pub_.getNumSubscribers() == 0);
  enable_active_ = !(enable_pub_.getNumSubscribers() == 0);
  velocity_mis_active_ = !(velocity_mis_pub_.getNumSubscribers() == 0);
  pose_active_ = !(pose_pub_.getNumSubscribers() == 0);
  motor_left_active_ = !(motor_left_pub_.getNumSubscribers() == 0);
  motor_right_active_ = !(motor_right_pub_.getNumSubscribers() == 0);
  time_process_active_ = !(time_process_pub_.getNumSubscribers() == 0);
  odom_active_ = !(odom_pub_.getNumSubscribers() == 0);
  joint_active_ = !(joint_pub_.getNumSubscribers() == 0);
  packet_ = updatePacket();
  //Update rate;
  int rate;
  //  std::string name_param = name_node_ + "/" + rate_update_string;
  nh_.getParam(name_node_ + "/" + rate_update_string, rate);
  //  ROS_INFO("rate paramam: %s, %d", name_param.c_str(), rate);
  if ((rate != rate_) && (rate < 100) && (rate > 0))
  {
    rate_ = rate;
    ros::Rate loop_rate(rate);
    loop_rate_ = loop_rate;
    ROS_INFO("Rate updated at: %d", rate_);
  }


  //  ROS_INFO("velocity_active_: %d", velocity_active_);
  //  ROS_INFO("enable_active_: %d", enable_active_);
  //  ROS_INFO("velocity_mis_active_: %d", velocity_mis_active_);
  //  ROS_INFO("pose_active_: %d", pose_active_);

  user_ = (velocity_active_ || enable_active_ || velocity_mis_active_ || pose_active_
    || motor_left_active_ || motor_right_active_ || time_process_active_ || odom_active_ || joint_active_);

  //  ROS_INFO("user: %d", user_);

  if (!user_)
  {
    ROS_INFO("Wait user");
    cond.wait(lock);
    user_ = true;
  }
  return user_;
}

void RosControllerSerial::loadParameter()
{
  ROS_INFO("Name node: %s", name_node_.c_str());
  if (nh_.hasParam(name_node_))
  {
    ROS_INFO("Sync parameter: ROS -> ROBOT");
    packet_t send_pkg;
    send_pkg.length = 0;
    constraint_t constraint = get_constraint();
    Serial::addPacket(&send_pkg, CONSTRAINT, CHANGE, (abstract_packet_t*) & constraint);
    pid_control_t pid = get_pid(left_string);
    Serial::addPacket(&send_pkg, PID_CONTROL_L, CHANGE, (abstract_packet_t*) & pid);
    pid = get_pid(right_string);
    Serial::addPacket(&send_pkg, PID_CONTROL_R, CHANGE, (abstract_packet_t*) & pid);
    parameter_t parameter = get_parameter();
    Serial::addPacket(&send_pkg, PARAMETER, CHANGE, (abstract_packet_t*) & parameter);

    Serial::parsing(NULL, serial_->sendPacket(send_pkg));

    nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + pwm_string, pwm_motor_);
    nh_.param<std::string>("/" + name_node_ + "/" + tf_string + "/" + odometry_string, tf_odometry_string_, tf_odometry_string_);
    nh_.param<std::string>("/" + name_node_ + "/" + tf_string + "/" + base_link_string, tf_base_link_string_, tf_base_link_string_);
    nh_.param<std::string>("/" + name_node_ + "/" + tf_string + "/" + joint_string, tf_joint_string_, tf_joint_string_);

    ROS_INFO("Write!");
  }
  else
  {
    ROS_INFO("Sync parameter: ROBOT -> ROS");
    tf_odometry_string_ = odometry_string;
    tf_base_link_string_ = base_link_string;
    tf_joint_string_ = joint_string;
    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_ele_string + "/" + left_string, 1.0);
    nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_ele_string + "/" + right_string, 1.0);
    nh_.setParam("/" + name_node_ + "/" + tf_string + "/" + odometry_string, tf_odometry_string_);
    nh_.setParam("/" + name_node_ + "/" + tf_string + "/" + base_link_string, tf_base_link_string_);
    nh_.setParam("/" + name_node_ + "/" + tf_string + "/" + base_link_string, tf_base_link_string_);
    nh_.setParam("/" + name_node_ + "/" + tf_string + "/" + joint_string, tf_joint_string_);
    init();
  }
}

void RosControllerSerial::init()
{
  packet_t packet;
  packet.length = 0;
  Serial::addPacket(&packet, CONSTRAINT, REQUEST, NULL);
  Serial::addPacket(&packet, PARAMETER, REQUEST, NULL);
  Serial::addPacket(&packet, PID_CONTROL_L, REQUEST, NULL);
  Serial::addPacket(&packet, PID_CONTROL_R, REQUEST, NULL);
  Serial::addPacket(&packet, CONSTRAINT, REQUEST, NULL);
  Serial::addPacket(&packet, PRIORITY_PROCESS, REQUEST, NULL);
  Serial::addPacket(&packet, FRQ_PROCESS, REQUEST, NULL);

  std::list<information_packet_t> configuration_robot;
  configuration_robot = Serial::parsing(NULL, serial_->sendPacket(packet));
  std::string name_pid;
  float step_timer, tm_mill, k_time;

  for (std::list<information_packet_t>::iterator list_iter = configuration_robot.begin(); list_iter != configuration_robot.end(); list_iter++)
  {
    information_packet_t packet = (*list_iter);
    if (packet.option == CHANGE)
    {
      switch (packet.command)
      {
        case PRIORITY_PROCESS:
          for (int i = 0; i < PROCESS_MOTION_LENGTH; i++)
          {
            nh_.setParam("/" + name_node_ + "/" + priority_string + "/" + string_process_motion[i], packet.packet.process.process[i]);
          }
          nh_.setParam("/" + name_node_ + "/" + priority_string + "/" + parse_string, packet.packet.process.parse_packet);
          break;
        case FRQ_PROCESS:
          for (int i = 0; i < PROCESS_MOTION_LENGTH; i++)
          {
            nh_.setParam("/" + name_node_ + "/" + frequency_string + "/" + string_process_motion[i], packet.packet.process.process[i]);
          }
          break;
        case CONSTRAINT:
          nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + constraint_string + "/" + right_string, packet.packet.constraint.max_right);
          nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + constraint_string + "/" + left_string, packet.packet.constraint.max_left);
          break;
        case PARAMETER:
          nh_.setParam("/" + name_node_ + "/" + structure_string + "/" + wheelbase_string, packet.packet.parameter.wheelbase);
          nh_.setParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + right_string, packet.packet.parameter.radius_r);
          nh_.setParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + left_string, packet.packet.parameter.radius_l);
          nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_vel_string + "/" + right_string, packet.packet.parameter.k_vel_r);
          nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_vel_string + "/" + left_string, packet.packet.parameter.k_vel_l);
          nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_ang_string + "/" + right_string, packet.packet.parameter.k_ang_r);
          nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + k_ang_string + "/" + left_string, packet.packet.parameter.k_ang_l);
          nh_.setParam("/" + name_node_ + "/" + joint_string + "/" + pwm_string, packet.packet.parameter.pwm_step);
          nh_.setParam("/" + name_node_ + "/" + sp_min_string, packet.packet.parameter.sp_min);
          step_timer = packet.packet.parameter.step_timer;
          tm_mill = packet.packet.parameter.int_tm_mill;
          k_time = 1 / (step_timer / tm_mill);
          pwm_motor_ = (int) packet.packet.parameter.pwm_step;
          nh_.setParam("/" + name_node_ + "/" + time_string + "/" + step_timer_string, step_timer);
          nh_.setParam("/" + name_node_ + "/" + time_string + "/" + int_tm_mill_string, tm_mill);
          nh_.setParam("/" + name_node_ + "/" + time_string + "/" + k_time_string, k_time);
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
      }
    }
  }
  nh_.setParam("/" + name_node_ + "/" + rate_update_string, rate_);
  // total space move robot
  nh_.setParam("/" + name_node_ + "/" + space_robot_string, 0);
  ROS_INFO("Saved");
}

float RosControllerSerial::correction_time_process(float process_time)
{
  if (process_time < 0)
  {
    double step_timer;
    nh_.getParam("/" + name_node_ + "/" + time_string + "/" + step_timer_string, step_timer);
    return step_timer + process_time;
  }
  return process_time;
}

void RosControllerSerial::th_stream()
{
  while (~(stream_bool() ^ !stream_exit_) && ros::ok())
  {
    if (packet_.length != 0)
    {
      std::list<information_packet_t> telemetry_serial;
      packet_t packet = serial_->sendPacket(packet_);
      telemetry_serial = Serial::parsing(NULL, packet);
      double rate = ((double) packet.time) / 1000000000;
      ros::Duration duration = ros::Duration(rate);
      serial_bridge::Pose pose;
      serial_bridge::Velocity velocity;
      serial_bridge::Motor motor_left, motor_right;
      for (std::list<information_packet_t>::iterator list_iter = telemetry_serial.begin(); list_iter != telemetry_serial.end(); list_iter++)
      {
        information_packet_t packet = (*list_iter);
        double k_time;
        if (packet.option == CHANGE)
        {
          serial_bridge::Enable enable_motors;
          serial_bridge::Process time_process;
          switch (packet.command)
          {
            case TIME_PROCESS:
              time_process.time = duration;
              nh_.getParam("/" + name_node_ + "/" + time_string + "/" + k_time_string, k_time);
              time_process.idle = k_time * correction_time_process(packet.packet.process.idle);
              time_process.parse_packet = k_time * correction_time_process(packet.packet.process.parse_packet);
              time_process.pid_l = k_time * correction_time_process(packet.packet.process.process[0]);
              time_process.pid_r = k_time * correction_time_process(packet.packet.process.process[1]);
              time_process.velocity = k_time * correction_time_process(packet.packet.process.process[2]);
              time_process.dead_reckoning = k_time * correction_time_process(packet.packet.process.process[3]);
              time_process_pub_.publish(time_process);
              break;
            case MOTOR_L:
              motor_left.time = duration;
              motor_left.rifer_vel = ((double) packet.packet.motor.rifer_vel) / 1000;
              motor_left.control_vel = -((double) packet.packet.motor.control_vel - ((double) pwm_motor_) / 2) / (((double) pwm_motor_) / 2);
              motor_left.measure_vel = ((double) packet.packet.motor.measure_vel) / 1000;
              motor_left.current = ((double) packet.packet.motor.current) / 1000;
              motor_left_pub_.publish(motor_left);
              break;
            case MOTOR_R:
              motor_right.time = duration;
              motor_right.rifer_vel = ((double) packet.packet.motor.rifer_vel) / 1000;
              motor_right.control_vel = ((double) packet.packet.motor.control_vel - ((double) pwm_motor_) / 2) / (((double) pwm_motor_) / 2);
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
              velocity.time = duration;
              enable_motors.enable = packet.packet.enable;
              //              ROS_INFO("ENABLE");
              enable_pub_.publish(enable_motors);
              break;
          }
        }
      }
      // Send Odometry message
      if (odom_active_)
      {
        //        ROS_INFO("SEND ODOM");
        sendOdom(velocity, pose);
      }
      if (joint_active_)
      {
        sendJoint(motor_left, motor_right);
      }
      ros::spinOnce();
    }
    loop_rate_.sleep();
  }
  ROS_INFO("close stream");
}

void RosControllerSerial::sendJoint(serial_bridge::Motor motor_left, serial_bridge::Motor motor_right)
{
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

void RosControllerSerial::sendOdom(serial_bridge::Velocity velocity, serial_bridge::Pose pose)
{
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

packet_t RosControllerSerial::updatePacket()
{
  packet_t packet;
  packet.length = 0;
  if (velocity_active_)
  {
    //    ROS_INFO("ADD VELOCITY");
    Serial::addPacket(&packet, VELOCITY, REQUEST, NULL);
  }
  if (velocity_mis_active_ || odom_active_)
  {
    //    ROS_INFO("ADD VELOCITY MIS");
    Serial::addPacket(&packet, VELOCITY_MIS, REQUEST, NULL);
  }
  if (motor_left_active_ || joint_active_)
  {
    //    ROS_INFO("ADD MOTOR LEFT");
    Serial::addPacket(&packet, MOTOR_L, REQUEST, NULL);
  }
  if (motor_right_active_ || joint_active_)
  {
    //    ROS_INFO("ADD MOTOR_RIGHT");
    Serial::addPacket(&packet, MOTOR_R, REQUEST, NULL);
  }
  if (enable_active_)
  {
    //    ROS_INFO("ADD ENABLE");
    Serial::addPacket(&packet, ENABLE, REQUEST, NULL);
  }
  if (pose_active_ || odom_active_)
  {
    //    ROS_INFO("ADD COORDINATE");
    Serial::addPacket(&packet, COORDINATE, REQUEST, NULL);
  }
  if (time_process_active_)
  {
    //    ROS_INFO("ADD COORDINATE");
    Serial::addPacket(&packet, TIME_PROCESS, REQUEST, NULL);
  }
  return packet;
}

void RosControllerSerial::connectCallback(const ros::SingleSubscriberPublisher& pub)
{
  //  ROS_INFO("callback: %s", pub.getTopic().c_str());
  if (((pose_pub_.getNumSubscribers() == 1) || (velocity_pub_.getNumSubscribers() == 1)
      || (enable_pub_.getNumSubscribers() == 1) || (velocity_mis_pub_.getNumSubscribers() == 1)
      || (motor_left_pub_.getNumSubscribers() == 1) || (motor_right_pub_.getNumSubscribers() == 1)
      || (time_process_pub_.getNumSubscribers() == 1) || (odom_pub_.getNumSubscribers() == 1)
      || (joint_pub_.getNumSubscribers() == 1)) && !user_)
  {
    cond.notify_one();
    ROS_INFO("Start streaming");
  }
}

void RosControllerSerial::velocityCallback(const serial_bridge::Velocity::ConstPtr &msg)
{
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

void RosControllerSerial::enableCallback(const serial_bridge::Enable::ConstPtr &msg)
{
  packet_t send_pkg;
  send_pkg.length = 0;
  enable_motor_t enable = msg->enable;
  Serial::addPacket(&send_pkg, ENABLE, CHANGE, (abstract_packet_t*) & enable);
  // TODO VERIFY THE PACKET
  Serial::parsing(NULL, serial_->sendPacket(send_pkg));
}

void RosControllerSerial::updateOdom(const serial_bridge::Pose* pose)
{
  float space;
  packet_t packet;
  packet.length = 0;
  Serial::addPacket(&packet, COORDINATE, REQUEST, NULL);
  std::list<information_packet_t> configuration_robot;
  configuration_robot = Serial::parsing(NULL, serial_->sendPacket(packet));
  for (std::list<information_packet_t>::iterator list_iter = configuration_robot.begin(); list_iter != configuration_robot.end(); list_iter++)
  {
    information_packet_t packet = (*list_iter);
    if (packet.option == CHANGE)
    {
      switch (packet.command)
      {
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

void RosControllerSerial::pose_tf_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  serial_bridge::Pose pose;
  pose.x = msg.get()->pose.pose.position.x;
  pose.y = msg.get()->pose.pose.position.y;
  pose.theta = tf::getYaw(msg.get()->pose.pose.orientation);
  updateOdom(&pose);
  ROS_INFO("Update initial position: [x: %f, y: %f, th: %f]", pose.x, pose.y, pose.theta);
}

void RosControllerSerial::poseCallback(const serial_bridge::Pose::ConstPtr &msg)
{
  //  ROS_INFO("POSE CALLBACK");
  updateOdom(msg.get());
}

pid_control_t RosControllerSerial::get_pid(std::string name)
{
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

parameter_t RosControllerSerial::get_parameter()
{
  parameter_t parameter;
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
  nh_.getParam("/" + name_node_ + "/" + time_string + "/" + step_timer_string, temp);
  parameter.step_timer = temp;
  nh_.getParam("/" + name_node_ + "/" + time_string + "/" + int_tm_mill_string, temp);
  parameter.int_tm_mill = temp;
  nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + pwm_string, temp_int);
  parameter.pwm_step = temp_int;
  return parameter;
}

constraint_t RosControllerSerial::get_constraint()
{
  constraint_t constraint;
  double temp;

  nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + constraint_string + "/" + right_string, temp);
  constraint.max_right = temp;
  nh_.getParam("/" + name_node_ + "/" + joint_string + "/" + constraint_string + "/" + left_string, temp);
  constraint.max_left = temp;
  return constraint;
}

process_t RosControllerSerial::get_process(std::string name)
{
  process_t process;
  int temp;
  process.idle = 0;
  for (int i = 0; i < PROCESS_MOTION_LENGTH; i++)
  {
    nh_.getParam("/" + name_node_ + "/" + name + "/" + string_process_motion[i], temp);
    process.process[i] = temp;
  }
  if (name.compare(priority_string) == 0)
  {
    nh_.getParam("/" + name_node_ + "/" + name + "/" + parse_string, temp);
    process.parse_packet = temp;
  }
  else process.parse_packet = 0;
  return process;
}

bool RosControllerSerial::parameter_update_Callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  packet_t send_pkg;
  send_pkg.length = 0;
  parameter_t parameter = get_parameter();
  Serial::addPacket(&send_pkg, PARAMETER, CHANGE, (abstract_packet_t*) & parameter);
  Serial::parsing(NULL, serial_->sendPacket(send_pkg));
  return true;
}

bool RosControllerSerial::process_update_Callback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&)
{
  std::string name = req.name;
  process_t process;
  packet_t send_pkg;
  send_pkg.length = 0;
  ROS_INFO("PROCESS UPDATE");
  if ((name.compare(priority_string) == 0) || (name.compare(all_string) == 0))
  {
    process = get_process(priority_string);
    Serial::addPacket(&send_pkg, PRIORITY_PROCESS, CHANGE, (abstract_packet_t*) & process);
  }
  if ((name.compare(frequency_string) == 0) || (name.compare(all_string) == 0))
  {
    process = get_process(frequency_string);
    Serial::addPacket(&send_pkg, FRQ_PROCESS, CHANGE, (abstract_packet_t*) & process);
  }
  if (send_pkg.length != 0)
  {
    // TODO VERIFY THE PACKET
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    return true;
  }
  else
  {
    ROS_ERROR("PROCESS ERROR UPDATE");
    return false;
  }
}

bool RosControllerSerial::pid_update_Callback(serial_bridge::Update::Request &req, serial_bridge::Update::Response&)
{
  std::string name = req.name;
  pid_control_t pid;
  packet_t send_pkg;
  send_pkg.length = 0;
  ROS_INFO("PID UPDATE");
  if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0))
  {
    pid = get_pid(left_string);
    Serial::addPacket(&send_pkg, PID_CONTROL_L, CHANGE, (abstract_packet_t*) & pid);
  }
  if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0))
  {
    pid = get_pid(right_string);
    Serial::addPacket(&send_pkg, PID_CONTROL_R, CHANGE, (abstract_packet_t*) & pid);
  }
  if (send_pkg.length != 0)
  {
    // TODO VERIFY THE PACKET
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    return true;
  }
  else
  {
    ROS_ERROR("PID ERROR UPDATE");
    return false;
  }
}

bool RosControllerSerial::constraint_update_Callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  packet_t send_pkg;
  send_pkg.length = 0;
  constraint_t constraint = get_constraint();
  Serial::addPacket(&send_pkg, CONSTRAINT, CHANGE, (abstract_packet_t*) & constraint);
  Serial::parsing(NULL, serial_->sendPacket(send_pkg));
  return true;
}

abstract_packet_t RosControllerSerial::getServiceSerial(std::list<information_packet_t> configuration, unsigned char command, unsigned char service_command)
{
  abstract_packet_t packet;
  //TODO control error to receive packet
  for (std::list<information_packet_t>::iterator list_iter = configuration.begin(); list_iter != configuration.end(); list_iter++)
  {
    information_packet_t packet = (*list_iter);
    if ((packet.option == CHANGE) && (packet.command = command))
    {
      if (command == SERVICES)
      {
        if (packet.packet.services.command == service_command)
          return packet.packet;
      }
      else
        return packet.packet;
    }
  }
  return packet;
}

bool RosControllerSerial::service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response & msg)
{
  packet_t send_pkg;
  std::stringstream service_str;
  send_pkg.length = 0;
  ROS_INFO("service: %s", req.name.c_str());
  if (req.name.compare(reset_string) == 0)
  {
    services_t service;
    service.command = RESET;
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & service);
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & service);
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & service);
    Serial::parsing(NULL, serial_->sendPacket(HEADER_ASYNC, send_pkg));
    msg.name = "reset";
  }
  else if (req.name.compare(version_string) == 0)
  {
    services_t version, author, name_board, date;
    version.command = VERSION_CODE;
    author.command = AUTHOR_CODE;
    name_board.command = NAME_BOARD;
    date.command = DATE_CODE;
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & version);
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & author);
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & name_board);
    Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & date);
    std::list<information_packet_t> configuration = Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    char* buff_vers = (char*) getServiceSerial(configuration, SERVICES, VERSION_CODE).services.buffer;
    char* buff_auth = (char*) getServiceSerial(configuration, SERVICES, AUTHOR_CODE).services.buffer;
    char* buff_name = (char*) getServiceSerial(configuration, SERVICES, NAME_BOARD).services.buffer;
    char* buff_date = (char*) getServiceSerial(configuration, SERVICES, DATE_CODE).services.buffer;
    std::string date_string(buff_date, SERVICE_BUFF);
    service_str << "Name Board: " << buff_name << " " << buff_vers << std::endl;
    service_str << buff_auth << " - Build in: " << date_string << std::endl;
    msg.name = service_str.str();
  }
  else if (req.name.compare(error_serial_string) == 0)
  {
    Serial::addPacket(&send_pkg, ERROR_SERIAL, REQUEST, NULL);
    std::list<information_packet_t> configuration = Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    int16_t* error_serial = getServiceSerial(configuration, ERROR_SERIAL, ' ').error_pkg.number;
    service_str << "Error list:" << std::endl;
    for (int i = 0; i < BUFF_SERIAL_ERROR; i++)
    {
      service_str << "Type: -" << (i + 1) << " - PC n: " << serial_->getBufferArray()[i] << " - PIC n: " << error_serial[i] << std::endl;
    }
    msg.name = service_str.str();
  }
  return true;
}

bool RosControllerSerial::convert_Callback(serial_bridge::Convert::Request &req, serial_bridge::Convert::Response & msg)
{
  double wheelbase, radius_r, radius_l;
  nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + wheelbase_string, wheelbase);
  if (req.type.compare(normalized_string) == 0)
  {
    msg.lin_vel = (req.vel_r + req.vel_l) / 2;
    msg.ang_vel = (req.vel_r - req.vel_l) / wheelbase;
    return true;
  }
  else if (req.type.compare(standard_string) == 0)
  {
    nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + right_string, radius_r);
    nh_.getParam("/" + name_node_ + "/" + structure_string + "/" + radius_string + "/" + left_string, radius_l);
    msg.lin_vel = (radius_r * req.vel_r + radius_l * req.vel_l) / 2;
    msg.ang_vel = (radius_r * req.vel_r - radius_l * req.vel_l) / wheelbase;
    return true;
  }
  else return false;

}