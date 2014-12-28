#include <ros/ros.h>
#include "std_msgs/String.h"

#include <geometry_msgs/Twist.h>
#include <serial_bridge/Enable.h>
#include <std_srvs/Empty.h>

#include <stdio.h>

class TeleopKeybrd
{
public:
	TeleopKeybrd(const ros::NodeHandle& nh, std::string robot, std::string command, std::string velocity, std::string enable);
  void keyLoop();

protected:
	std::string  enable_string_convert(bool enable);
	void enable(char c, char command);
	void enable_Callback(const serial_bridge::Enable::ConstPtr& msg);

private:
  ros::NodeHandle m_nh;

  double mLinear;
  double mAngular;
  double mMaxLin;
  double mMaxAng;

  double mLinStep;
  double mAngStep;

  double mSpeedRatio;

  int mKeyTimeout;

  ros::Publisher mVelPub;

	bool mLocked;

	ros::Publisher mPubVelControl, mPubEnableControl;
	ros::Subscriber mSubEnable;

	serial_bridge::Enable mEnablePkg;
};


