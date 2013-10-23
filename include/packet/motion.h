/* 
 * File:   motion.h
 * Author: raffaello
 *
 * Created on 27 September 2013, 18:45
 */

#ifndef MOTION_H
#define	MOTION_H

#define PID_CONTROL_L 'A'
#define PID_CONTROL_R 'B'
#define MOTOR_L 'a'
#define MOTOR_R 'b'
#define COORDINATE 'C'
#define PARAMETER 'P'
#define CONSTRAINT 'T'
#define VELOCITY 'V'
#define VELOCITY_MIS 'M'
#define ENABLE 'E'

#define PROCESS_MOTION_LENGTH 4
#define PROCESS_PID_LEFT 0
#define PID_LEFT_STRING "PID/Left"
#define PROCESS_PID_RIGHT 1
#define PID_RIGHT_STRING "PID/Right"
#define PROCESS_VELOCITY 2
#define VELOCITY_STRING "Velocity"
#define PROCESS_ODOMETRY 3
#define ODOMETRY_STRING "Odometry"

typedef struct constraint {
    float max_left;
    float max_right;
} constraint_t;
#define LNG_CONSTRAINT sizeof(motor_t)

typedef struct motor {
    int16_t rifer_vel;
    int16_t control_vel;
    int16_t measure_vel;
    int16_t current;
} motor_t;
#define LNG_MOTOR sizeof(motor_t)

typedef struct pid {
    float kp;
    float ki;
    float kd;
} pid_control_t;
#define LNG_PID_CONTROL sizeof(pid_control_t)

typedef struct coordinate {
    float x;
    float y;
    float theta;
    float space;
} coordinate_t;
#define LNG_COORDINATE sizeof(coordinate_t)

typedef struct parameter {
    float radius_r;
    float radius_l;
    float wheelbase;
    float k_vel_r;
    float k_vel_l;
    float k_ang_r;
    float k_ang_l;
    float sp_min;
    int16_t pwm_step;
    int16_t step_timer;
    int16_t int_tm_mill;
} parameter_t;
#define LNG_PARAMETER sizeof(parameter_t)

typedef struct velocity {
    float v;
    float w;
} velocity_t;
#define LNG_VELOCITY sizeof(velocity_t)

typedef uint8_t enable_motor_t;
#define LNG_ENABLE sizeof(enable_motor_t) + 1

#define ABSTRACT_PACKET_MOTION      \
        pid_control_t pid;          \
        coordinate_t coordinate;    \
        parameter_t parameter;      \
        velocity_t velocity;        \
        enable_motor_t enable;      \
        motor_t motor;              \
        constraint_t constraint;    \


#endif	/* MOTION_H */

