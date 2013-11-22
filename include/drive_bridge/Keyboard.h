/* 
 * File:   Keyboard.h
 * Author: raffaello
 *
 * Created on 16 September 2013, 16:15
 */

#ifndef KEYBOARD_H
#define	KEYBOARD_H

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <geometry_msgs/Twist.h>
#include <serial_bridge/Enable.h>
#include <std_srvs/Empty.h>

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_BACKSPACE 0x7F
#define KEYCODE_ENTER 0x0A

const double default_step = 0.1;

class Keyboard {
public:
    Keyboard(const ros::NodeHandle& nh, std::string robot, std::string command, std::string velocity, std::string enable);
    Keyboard(const Keyboard& orig);
    virtual ~Keyboard();
    void read_keyboard();
    void quit(int sig);
private:
    ros::NodeHandle nh_;
    bool receive_command_;
    int kfd;
    struct termios cooked, raw;
    void arrows(char c);
    void letters(char c);
    void enable(char c, char command);
    void step(char c);
    void enable_Callback(const serial_bridge::Enable::ConstPtr& msg);
    std::string enable_string_convert(bool enable);
    
    typedef struct step {
        double step_lin;
        double step_ang;
    } step_t;
    
    step_t step_;
    
    ros::Publisher pub_vel_control_, pub_enable_control_;
    ros::Subscriber sub_enable_;
    
    serial_bridge::Enable enable_pkg;
    geometry_msgs::Twist velocity;
};

#endif	/* KEYBOARD_H */
