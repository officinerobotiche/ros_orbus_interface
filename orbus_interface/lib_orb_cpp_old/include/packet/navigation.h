/* 
 * File:   navigation.h
 * Author: raffaello
 *
 * Created on 27 September 2013, 18:47
 */

#ifndef NAVIGATION_H
#define	NAVIGATION_H

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
    int8_t pkgs[BUFFER_AUTOSEND];
} autosend_t;
#define LNG_AUTOSEND sizeof(autosend_t)

typedef uint8_t enable_sensor_t;
#define LNG_ENABLE_SENSOR sizeof(enable_sensor_t)

#define ABSTRACT_MESSAGE_NAVIGATION             \
        sensor_t sensor;                        \
        humidity_t humidity;                    \
        infrared_t infrared;                    \
        parameter_sensor_t parameter_sensor;    \
        autosend_t enable_autosend;             \
        enable_sensor_t enable_sensor;          \
        

#define SENSOR 0
#define INFRARED 1
#define HUMIDITY 2
#define PARAMETER_SENSOR 3
#define ENABLE_AUTOSEND 4
#define ENABLE_SENSOR 5

#define PROCESS_SENSOR_LENGTH 2
#define PROCESS_ADC_SENSOR 0
#define ADC_SENSOR_STRING "adc_convert"
#define PROCESS_SENDER 1
#define SENDER_STRING "sender"

#define HASHMAP_NAVIGATION 'N'
static unsigned int hashmap_navigation[10];

#define INITIALIZE_HASHMAP_NAVIGATION hashmap_navigation[SENSOR] = LNG_SENSOR;    \
                                      hashmap_navigation[INFRARED] = LNG_INFRARED; \
                                      hashmap_navigation[HUMIDITY] = LNG_HUMIDITY; \
                                      hashmap_navigation[PARAMETER_SENSOR] = LNG_PARAMETER_SENSOR; \
                                      hashmap_navigation[ENABLE_AUTOSEND] = LNG_AUTOSEND; \
                                      hashmap_navigation[ENABLE_SENSOR] = LNG_ENABLE_SENSOR;

#endif	/* NAVIGATION_H */

