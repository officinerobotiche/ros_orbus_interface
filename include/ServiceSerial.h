/* 
 * File:   ServiceSerial.h
 * Author: raffaello
 *
 * Created on 26 October 2013, 14:43
 */

#ifndef SERVICESERIAL_H
#define	SERVICESERIAL_H

#include "Serial.h"

class ServiceSerial {
public:
    ServiceSerial(Serial* serial);
    ServiceSerial(const ServiceSerial& orig);
    virtual ~ServiceSerial();
    std::string getNameBoard();
    std::string getAuthor();
    std::string getCompiled();
    std::string getVersion();
private:
    Serial* serial_; //Serial object to comunicate with PIC device
    std::string name_board_string, version_string, name_author_string, compiled_string;
    abstract_packet_t getServiceSerial(std::list<information_packet_t> configuration, unsigned char command, unsigned char service_command);
};

#endif	/* SERVICESERIAL_H */

