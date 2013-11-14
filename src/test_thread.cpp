
#include <ros/ros.h>
#include "async_serial/ParserPacket.h"
#include "serial_controller/ROSController.h"

using namespace std;

template<class T = packet_t, int N = 10 >
class test {
public:

    test() {
    };

    typedef boost::function<void (const T*) > callback_t;
    boost::array<callback_t, N > async_functions;
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "threadtest");

    ros::NodeHandle nh;

    ROS_INFO("Start");

    ParserPacket serial("/dev/ttyUSB0", 115200);

    packet_t packet;
    std::vector<information_packet_t> list_send, list_return;
    services_t version, author, name_board, date;
    version.command = VERSION_CODE;
//    list_send.push_back(serial.createPacket(SERVICES, HASHMAP_DEFAULT, (abstract_packet_t*) & version));
//
//    packet = serial.sendSyncPacket(packet);
//
//    for (std::vector<information_packet_t>::iterator list_iter = list_send.begin(); list_iter != list_send.end(); list_iter++) {
//        information_packet_t packet = (*list_iter);
//        ROS_INFO("packet: %c, %c", packet.command, packet.option);
//        //ROS_INFO("packet: %d, %d", packet.command, packet.option); 
//    }

    //    packet.length = 0;
    //    serial.addPacket(&packet, CONSTRAINT, REQUEST, NULL);
    //
    //    enable_motor_t enable = false;
    //    serial.addPacket(&packet, ENABLE, CHANGE, (abstract_packet_t*) & enable);

    //    packet = serial.sendSyncPacket(packet);
    //
    //    ROS_INFO("length: %d", packet.length);
    //    
    //    std::list<information_packet_t> serial_packet = serial.parsing(packet);
    //    for (std::list<information_packet_t>::iterator list_iter = serial_packet.begin(); list_iter != serial_packet.end(); list_iter++) {
    //        information_packet_t packet = (*list_iter);
    //        ROS_INFO("packet: %c, %c", packet.command, packet.option);
    //        //ROS_INFO("packet: %d, %d", packet.command, packet.option); 
    //    }
    //
    //    ROSController controller("test", nh, &serial);
    //
    //    test<std::list<information_packet_t> > testa();
    //    test* tst = new test<int>();
    //    tst->number = 1;

    //    ros::spin();
}