/* 
 * File:   Keyboard.cpp
 * Author: raffaello
 * 
 * Created on 16 September 2013, 16:15
 */

#include "drive_bridge/Keyboard.h"

Keyboard::Keyboard(const ros::NodeHandle& nh, std::string robot, std::string command, std::string velocity, std::string enable)
: nh_(nh)
{
  pub_vel_control_ = nh_.advertise<geometry_msgs::Twist>("/" + robot + "/" + command + "/" + velocity, 1000);
  pub_enable_control_ = nh_.advertise<serial_bridge::Enable>("/" + robot + "/" + command + "/" + enable, 1000);
  sub_enable_ = nh_.subscribe("/" + robot + "/" + enable, 1000, &Keyboard::enable_Callback, this);
  kfd = 0;
  receive_command_ = false;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof (struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  step_.step_lin = 0.1;
  step_.step_ang = 0.1;
}

Keyboard::Keyboard(const Keyboard& orig)
{
}

Keyboard::~Keyboard()
{
}

std::string Keyboard::enable_string_convert(bool enable)
{
  return ((enable) ? "ON" : "OFF");
}

void Keyboard::arrows(char c)
{
  switch (c)
  {
    case KEYCODE_L:
      ROS_DEBUG("LEFT");
      velocity.angular.z += step_.step_ang;
      receive_command_ = true;
      break;
    case KEYCODE_R:
      ROS_DEBUG("RIGHT");
      velocity.angular.z -= step_.step_ang;
      receive_command_ = true;
      break;
    case KEYCODE_U:
      ROS_DEBUG("UP");
      velocity.linear.x += step_.step_lin;
      receive_command_ = true;
      break;
    case KEYCODE_D:
      ROS_DEBUG("DOWN");
      velocity.linear.x -= step_.step_lin;
      receive_command_ = true;
      break;
  }
}

void Keyboard::letters(char c)
{
  switch (c)
  {
    case 'a':
      ROS_DEBUG("LEFT");
      velocity.angular.z += step_.step_ang;
      receive_command_ = true;
      break;
    case 'd':
      ROS_DEBUG("RIGHT");
      velocity.angular.z -= step_.step_ang;
      receive_command_ = true;
      break;
    case 'w':
      ROS_DEBUG("UP");
      velocity.linear.x += step_.step_lin;
      receive_command_ = true;
      break;
    case 's':
      ROS_DEBUG("DOWN");
      velocity.linear.x -= step_.step_lin;
      receive_command_ = true;
      break;
  }
}

void Keyboard::enable_Callback(const serial_bridge::Enable::ConstPtr& msg)
{
  enable_pkg.enable = msg.get()->enable;
  ROS_INFO("Enable now: %s", enable_string_convert(enable_pkg.enable).c_str());
  sub_enable_.shutdown();
}

void Keyboard::enable(char c, char command)
{
  if (c == command)
  {
    enable_pkg.enable = enable_pkg.enable ? false : true;
    ROS_INFO("Enable: %s", enable_string_convert(enable_pkg.enable).c_str());
    pub_enable_control_.publish(enable_pkg);
  }
}

void Keyboard::step(char c)
{
  switch (c)
  {
    case '+':
      step_.step_lin += default_step;
      step_.step_ang += default_step;
      ROS_INFO("step: [%f, %f]", step_.step_lin, step_.step_ang);
      break;
    case '-':
      step_.step_lin -= default_step;
      step_.step_ang -= default_step;
      if (step_.step_lin <= default_step)
        step_.step_lin = default_step;
      if (step_.step_ang <= default_step)
        step_.step_ang = default_step;
      ROS_INFO("step: [%f, %f]", step_.step_lin, step_.step_ang);
      break;
  }

}

void Keyboard::read_keyboard()
{
  char c;
  std_srvs::Empty close;
  while (ros::ok())
  {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

    step(c);
    arrows(c);
    letters(c);
    enable(c, '/');

    switch (c)
    {
      case ' ':
      case '*':
        ROS_DEBUG("SPACE");
        velocity.linear.x = 0;
        velocity.angular.z = 0;
        receive_command_ = true;
        break;
    }

    if (receive_command_)
    {
      ROS_INFO("Command [%f, %f]", velocity.linear.x, velocity.angular.z);
      pub_vel_control_.publish(velocity);
      receive_command_ = false;
    }
  }
}

void Keyboard::quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
}