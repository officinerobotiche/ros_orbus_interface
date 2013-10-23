/* 
 * File:   navigation.h
 * Author: raffaello
 *
 * Created on 27 September 2013, 18:47
 */

#ifndef NAVIGATION_H
#define	NAVIGATION_H

    typedef struct infrared {
        int16_t infrared;
    } infrared_t;

#define ABSTRACT_PACKET_NAVIGATION  \
        infrared_t infrared;        \

#endif	/* NAVIGATION_H */

