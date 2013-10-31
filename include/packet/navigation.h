/* 
 * File:   navigation.h
 * Author: raffaello
 *
 * Created on 27 September 2013, 18:47
 */

#ifndef NAVIGATION_H
#define	NAVIGATION_H

#define NUMBER_INFRARED 7

    typedef struct infrared {
        int16_t infrared[NUMBER_INFRARED];
    } infrared_t;
    #define LNG_INFRARED sizeof(infrared_t)

#define ABSTRACT_PACKET_NAVIGATION  \
        infrared_t infrared;        \

#endif	/* NAVIGATION_H */

