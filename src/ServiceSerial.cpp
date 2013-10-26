/* 
 * File:   ServiceSerial.cpp
 * Author: raffaello
 * 
 * Created on 26 October 2013, 14:43
 */

#include "ServiceSerial.h"

ServiceSerial::ServiceSerial(Serial* serial)
{
  this->serial_ = serial; // Initialize serial port
  packet_t send_pkg;
  send_pkg.length = 0;
  services_t version, author, name_board, date;
  version.command = VERSION_CODE;
  author.command = AUTHOR_CODE;
  name_board.command = NAME_BOARD;
  date.command = DATE_CODE;
  Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & version);
  Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & author);
  Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & name_board);
  Serial::addPacket(&send_pkg, SERVICES, CHANGE, (abstract_packet_t*) & date);
  ROS_INFO("PKG lng: %d, buff: %s", send_pkg.length, send_pkg.buffer);
  std::list<information_packet_t> configuration = Serial::parsing(NULL, serial->sendPacket(send_pkg));
  ////  char* buff_vers = (char*) getServiceSerial(configuration, SERVICES, VERSION_CODE).services.buffer;
  ////  char* buff_auth = (char*) getServiceSerial(configuration, SERVICES, AUTHOR_CODE).services.buffer;
  ////  char* buff_name = (char*) getServiceSerial(configuration, SERVICES, NAME_BOARD).services.buffer;
  ////  char* buff_date = (char*) getServiceSerial(configuration, SERVICES, DATE_CODE).services.buffer;

  //  this->name_board.append(buff_name);
  //  this->name_author.append(buff_auth);
  //  this->version.append(buff_vers);
  //  this->compiled.append(buff_date, SERVICE_BUFF);
  //  std::string date_string(buff_date, SERVICE_BUFF);
}

ServiceSerial::ServiceSerial(const ServiceSerial& orig)
{
}

ServiceSerial::~ServiceSerial()
{
}

abstract_packet_t ServiceSerial::getServiceSerial(std::list<information_packet_t> configuration, unsigned char command, unsigned char service_command)
{
  abstract_packet_t packet;
  //TODO control error to receive packet
  for (std::list<information_packet_t>::iterator list_iter = configuration.begin(); list_iter != configuration.end(); list_iter++)
  {
    information_packet_t packet = (*list_iter);
    if ((packet.option == CHANGE) && (packet.command = command))
    {
      if (command == SERVICES)
      {
        if (packet.packet.services.command == service_command)
          return packet.packet;
      }
      else
        return packet.packet;
    }
  }
  return packet;
}

std::string ServiceSerial::getNameBoard()
{
  return name_board_string;
}

std::string ServiceSerial::getAuthor()
{
  return name_author_string;
}

std::string ServiceSerial::getCompiled()
{
  return compiled_string;
}

std::string ServiceSerial::getVersion()
{
  return version_string;
}