/* 
 * File:   navigation.h
 * Author: raffaello
 *
 * Created on 27 September 2013, 18:47
 */

#ifndef NAVIGATION_H
#define	NAVIGATION_H

#define ENABLE_SENSOR 'E'

#define NUMBER_INFRARED 7

    typedef struct infrared {
        int16_t infrared[NUMBER_INFRARED];
    } infrared_t;
    #define LNG_INFRARED sizeof(infrared_t)

typedef uint8_t enable_sensor_t;
#define LNG_ENABLE_SENSOR sizeof(enable_sensor_t) + 1

#define ABSTRACT_PACKET_NAVIGATION      \
        infrared_t infrared;            \
        enable_sensor_t enable_sensor;  \

#endif	/* NAVIGATION_H */

