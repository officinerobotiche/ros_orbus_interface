/* 
 * File:   packet.h
 * Author: raffaello
 *
 * Created on June 2, 2013, 5:46 PM
 */

#ifndef PACKET_H
#define	PACKET_H

/** Serial **/

#include <stdint.h>

#define MOTION_CONTROL
#define NAVIGATION_BOARD

/*******/

#ifdef MOTION_CONTROL
#include "motion.h"
#endif
#ifdef NAVIGATION_BOARD
#include "navigation.h"
#endif

#define MAX_TX_BUFF 200
#define MAX_RX_BUFF 200
#define BUFF_SERIAL_ERROR 13
#define BUFF_ALL_PROCESS 10

#define REQUEST 'R'
#define CHANGE 'C'
#define ACK 'K'
#define NACK 'N'

//

#define TIME_PROCESS 't'
#define PARAMETER_SYSTEM 'm'
#define PRIORITY_PROCESS 'p'
#define FRQ_PROCESS 'f'
#define ERROR_SERIAL 'e'

#define SERVICES 's'
#define SERVICE_BUFF 20
#define RESET '*'
#define DATE_CODE 'd'
#define NAME_BOARD 'n'
#define VERSION_CODE 'v'
#define AUTHOR_CODE 'a'

#define LNG_ENABLE 2

typedef struct parameter_system {
    int16_t step_timer;
    int16_t int_tm_mill;
} parameter_system_t;
#define LNG_PARAMETER_SYSTEM sizeof(parameter_system_t)

typedef struct error_pkg {
    int16_t number[BUFF_SERIAL_ERROR];
} error_pkg_t;
#define LNG_ERROR_PKG sizeof(error_pkg_t)

typedef struct services {
    char command;
    unsigned char buffer[SERVICE_BUFF];
} services_t;
#define LNG_SERVICES sizeof(services_t)

typedef struct process {
    int16_t idle;
    int16_t parse_packet;
    int16_t process[BUFF_ALL_PROCESS];
} process_t;
#define LNG_PROCESS sizeof(process_t)

typedef struct packet_data {
    unsigned int length;
    unsigned char buffer[MAX_RX_BUFF];
    unsigned int time;
} packet_t;

typedef union abstract_packet {
    unsigned char buffer[MAX_RX_BUFF];
    process_t process;
    services_t services;
    error_pkg_t error_pkg;
    parameter_system_t parameter_system;
#ifdef MOTION_CONTROL
    ABSTRACT_PACKET_MOTION
#endif
#ifdef NAVIGATION_BOARD
    ABSTRACT_PACKET_NAVIGATION
#endif
} abstract_packet_t;
typedef abstract_packet_t* Ptr_abstract_packet;

typedef struct information_packet {
    unsigned char command;
    unsigned char option;
    abstract_packet_t packet;
} information_packet_t;

typedef packet_t* ppacket;

#endif	/* PACKET_H */

