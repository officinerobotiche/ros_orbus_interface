/**
*
*  \author     Raffaello Bonghi <raffaello.bonghi@officinerobotiche.it>
*  \copyright  Copyright (c) 2014-2015, Officine Robotiche, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "configurator/MotorParamConfigurator.h"

using namespace std;

#define PARAM_BRIDGE_STRING "/bridge"
#define PARAM_ENCODER_STRING "/encoder"

MotorParamConfigurator::MotorParamConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, std::string name, unsigned int number)
    : GenericConfigurator(nh, serial, number)
{
    // Find path param
    mName = nh_.getNamespace() + "/" + name;
    // Set command message
    mCommand.bitset.command = MOTOR_PARAMETER;

    setup_param = false;
    setup_encoder = false;
    setup_bridge = false;

    //Load dynamic reconfigure
    ds_param = new dynamic_reconfigure::Server<orbus_interface::UnavParameterConfig>(ros::NodeHandle(mName));
    dynamic_reconfigure::Server<orbus_interface::UnavParameterConfig>::CallbackType cb_param = boost::bind(&MotorParamConfigurator::reconfigureCBParam, this, _1, _2);
    ds_param->setCallback(cb_param);

    ds_encoder = new dynamic_reconfigure::Server<orbus_interface::UnavEncoderConfig>(ros::NodeHandle(mName + PARAM_ENCODER_STRING));
    dynamic_reconfigure::Server<orbus_interface::UnavEncoderConfig>::CallbackType cb_encoder = boost::bind(&MotorParamConfigurator::reconfigureCBEncoder, this, _1, _2);
    ds_encoder->setCallback(cb_encoder);

    ds_bridge = new dynamic_reconfigure::Server<orbus_interface::UnavBridgeConfig>(ros::NodeHandle(mName + PARAM_BRIDGE_STRING));
    dynamic_reconfigure::Server<orbus_interface::UnavBridgeConfig>::CallbackType cb_bridge = boost::bind(&MotorParamConfigurator::reconfigureCBBridge, this, _1, _2);
    ds_bridge->setCallback(cb_bridge);
}

void MotorParamConfigurator::initConfigurator()
{    
    /// Send configuration to board
    message_abstract_u temp;
    temp.motor.parameter = getParam();
    /// Call the function in Generic Reconfigurator
    SendParameterToBoard(temp);
}

void MotorParamConfigurator::setParam(motor_parameter_t parameter) {
    motor_parameter_encoder_t encoder = parameter.encoder;
    motor_parameter_bridge_t bridge = parameter.bridge;
    nh_.setParam(mName + "/ratio", (double) parameter.ratio);
    nh_.setParam(mName + "/rotation", (int) parameter.rotation);

    nh_.setParam(mName + PARAM_BRIDGE_STRING + "/h_bridge_enable", (int) bridge.enable);
    nh_.setParam(mName + PARAM_BRIDGE_STRING + "/PWM_dead_zone", (int) bridge.pwm_dead_zone);
    nh_.setParam(mName + PARAM_BRIDGE_STRING + "/PWM_frequency", (int) bridge.pwm_frequency);
    nh_.setParam(mName + PARAM_BRIDGE_STRING + "/volt_offset", (double) bridge.volt_offset);
    nh_.setParam(mName + PARAM_BRIDGE_STRING + "/volt_gain", (double) bridge.volt_gain);
    nh_.setParam(mName + PARAM_BRIDGE_STRING + "/current_offset", (double) bridge.current_offset);
    nh_.setParam(mName + PARAM_BRIDGE_STRING + "/current_gain", (double) bridge.current_gain);

    nh_.setParam(mName + PARAM_ENCODER_STRING + "/CPR", (int) encoder.cpr);
    nh_.setParam(mName + PARAM_ENCODER_STRING + "/position", (int) encoder.type.position);
    nh_.setParam(mName + PARAM_ENCODER_STRING + "/z_index", (int) encoder.type.z_index);
    nh_.setParam(mName + PARAM_ENCODER_STRING + "/channels", (int) (encoder.type.channels + 1));
}

motor_parameter_t MotorParamConfigurator::getParam() {
    motor_parameter_t parameter;
    int temp_int;
    double temp_double;
    nh_.getParam(mName + "/ratio", temp_double);
    parameter.ratio = (float) temp_double;
    nh_.getParam(mName + "/rotation", temp_int);
    parameter.rotation = (int8_t) temp_int;
    ROS_DEBUG_STREAM("Read param from " << mName << " [Ratio:" << parameter.ratio << ", Rotation:" << (int) parameter.rotation << "]");

    nh_.getParam(mName + PARAM_BRIDGE_STRING + "/h_bridge_enable", temp_int);
    parameter.bridge.enable = (uint8_t) temp_int;
    nh_.getParam(mName + PARAM_BRIDGE_STRING + "/PWM_dead_zone", temp_int);
    parameter.bridge.pwm_dead_zone = (uint16_t) temp_int;
    nh_.getParam(mName + PARAM_BRIDGE_STRING + "/PWM_frequency", temp_int);
    parameter.bridge.pwm_frequency = (uint16_t) temp_int;
    nh_.getParam(mName + PARAM_BRIDGE_STRING + "/volt_offset", temp_double);
    parameter.bridge.volt_offset = (float) temp_double;
    nh_.getParam(mName + PARAM_BRIDGE_STRING + "/volt_gain", temp_double);
    parameter.bridge.volt_gain = (float) temp_double;
    nh_.getParam(mName + PARAM_BRIDGE_STRING + "/current_offset", temp_double);
    parameter.bridge.current_offset = (float) temp_double;
    nh_.getParam(mName + PARAM_BRIDGE_STRING + "/current_gain", temp_double);
    parameter.bridge.current_gain = (float) temp_double;

    ROS_DEBUG_STREAM("Read param from " << mName << " [Enable:" << (int) parameter.bridge.enable
                    << ", PWM_Dead_zone:" << parameter.bridge.pwm_dead_zone
                    << ", PWM_Frequency:" << parameter.bridge.pwm_frequency
                    << ", Volt_Offset:" << parameter.bridge.volt_offset
                    << ", Volt_Gain:" << parameter.bridge.volt_gain
                    << ", Current_Offset:" << parameter.bridge.current_offset
                    << ", Current_Gain:" << parameter.bridge.current_gain << "]");

    nh_.getParam(mName + PARAM_ENCODER_STRING + "/CPR", temp_int);
    parameter.encoder.cpr = (uint16_t) temp_int;
    nh_.getParam(mName + PARAM_ENCODER_STRING + "/position", temp_int);
    parameter.encoder.type.position = (uint8_t) temp_int;
    nh_.getParam(mName + PARAM_ENCODER_STRING + "/z_index", temp_int);
    parameter.encoder.type.z_index = (uint8_t) temp_int;
    nh_.getParam(mName + PARAM_ENCODER_STRING + "/channels", temp_int);
    parameter.encoder.type.channels = (uint8_t) (temp_int - 1);

    ROS_DEBUG_STREAM("Read param from " << mName << " [CPR:" << parameter.encoder.cpr
                    << ", Position:" << (int) parameter.encoder.type.position
                    << ", Z_index:" << (int) parameter.encoder.type.z_index
                    << ", Channels:" << (int) parameter.encoder.type.channels << "]");

    return parameter;
}

void MotorParamConfigurator::reconfigureCBParam(orbus_interface::UnavParameterConfig &config, uint32_t level) {

    parameter.ratio = (float) config.ratio;
    parameter.rotation = (int8_t) config.rotation;

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_param)
    {
      last_param_ = parameter;
      default_param_ = last_param_;
      setup_param = true;
      return;
    }

    /// Send configuration to board
    message_abstract_u temp;
    temp.motor.parameter = parameter;
    /// Call the function in Generic Reconfigurator
    SendParameterToBoard(temp);

    // Store last parameter data
    last_param_ = parameter;
}

void MotorParamConfigurator::reconfigureCBEncoder(orbus_interface::UnavEncoderConfig &config, uint32_t level) {

    parameter.encoder.cpr = (uint16_t) config.CPR;
    parameter.encoder.type.position = (uint8_t) config.position;
    parameter.encoder.type.z_index = (uint8_t) config.z_index;
    parameter.encoder.type.channels = (uint8_t) (config.channels - 1);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_encoder)
    {
      last_param_ = parameter;
      default_param_ = last_param_;
      setup_encoder = true;
      return;
    }

    /// Send configuration to board
    message_abstract_u temp;
    temp.motor.parameter = parameter;
    /// Call the function in Generic Reconfigurator
    SendParameterToBoard(temp);

    // Store last parameter data
    last_param_ = parameter;
}

void MotorParamConfigurator::reconfigureCBBridge(orbus_interface::UnavBridgeConfig &config, uint32_t level) {

    parameter.bridge.enable = (uint8_t) config.h_bridge_enable;
    parameter.bridge.pwm_dead_zone = (uint16_t) config.PWM_dead_zone;
    parameter.bridge.pwm_frequency = (uint16_t) config.PWM_frequency;
    parameter.bridge.volt_offset = (float) config.volt_offset;
    parameter.bridge.volt_gain = (float) config.volt_gain;
    parameter.bridge.current_offset = (float) config.current_offset;
    parameter.bridge.current_gain = (float) config.current_gain;

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_bridge)
    {
      last_param_ = parameter;
      default_param_ = last_param_;
      setup_bridge = true;
      return;
    }

    /// Send configuration to board
    message_abstract_u temp;
    temp.motor.parameter = parameter;
    /// Call the function in Generic Reconfigurator
    SendParameterToBoard(temp);

    // Store last parameter data
    last_param_ = parameter;
}
