/* 
 * File:   navigation.h
 * Author: raffaello
 *
 * Created on 27 September 2013, 18:47
 */

#ifndef NAVIGATION_H
#define	NAVIGATION_H

#define SENSOR 'S'
#define INFRARED 'F'
#define HUMIDITY 'H'
#define PARAMETER_SENSOR 'R'
#define ENABLE_AUTOSEND 'D'
#define ENABLE_SENSOR 'N'

#define PROCESS_SENSOR_LENGTH 2
#define PROCESS_ADC_SENSOR 0
#define ADC_SENSOR_STRING "adc_convert"
#define PROCESS_SENDER 1
#define SENDER_STRING "sender"

#define NUMBER_INFRARED 7
#define BUFFER_AUTOSEND 10

    typedef struct sensor {
        float temperature;
        float voltage;
        float current;
    } sensor_t;
#define LNG_SENSOR sizeof(sensor_t)

    typedef float humidity_t;
#define LNG_HUMIDITY sizeof(humidity_t)

    typedef struct infrared {
        float infrared[NUMBER_INFRARED];
    } infrared_t;
#define LNG_INFRARED sizeof(infrared_t)

    typedef struct parameter_sensor {
        float gain_sharp;
        float exp_sharp;
        float gain_temperature;
        float gain_voltage;
        float gain_current;
        float gain_humidity;
    } parameter_sensor_t;
#define LNG_PARAMETER_SENSOR sizeof(parameter_sensor_t)

    typedef struct autosend {
        char pkgs[BUFFER_AUTOSEND];
    } autosend_t;
#define LNG_AUTOSEND sizeof(autosend_t)
    
    typedef uint8_t enable_sensor_t;

#define ABSTRACT_PACKET_NAVIGATION              \
        sensor_t sensor;                        \
        humidity_t humidity;                    \
        infrared_t infrared;                    \
        parameter_sensor_t parameter_sensor;    \
        autosend_t enable_autosend;             \
        enable_sensor_t enable_sensor;          \

#endif	/* NAVIGATION_H */

