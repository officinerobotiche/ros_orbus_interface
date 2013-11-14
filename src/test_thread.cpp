
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

    std::vector<information_packet_t> list_send, list_return;

    velocity_t velocity;
    velocity.v = 0;
    velocity.w = 0.2;

    list_send.push_back(serial.createPacket(VELOCITY, REQUEST, HASHMAP_MOTION));
    list_send.push_back(serial.createDataPacket(VELOCITY, HASHMAP_MOTION, (abstract_packet_t*) & velocity));
    list_send.push_back(serial.createPacket(VELOCITY, REQUEST, HASHMAP_MOTION));
    list_send.push_back(serial.createPacket(120, REQUEST, HASHMAP_MOTION));

    packet_t send = serial.encoder(list_send);

    packet_t receive = serial.sendSyncPacket(send);

    ROS_INFO("Send");
    for (std::vector<information_packet_t>::iterator list_iter = list_send.begin(); list_iter != list_send.end(); ++list_iter) {
        information_packet_t packet = (*list_iter);
        ROS_INFO("Cmd: %d - Type: %c - Option: %c - lng: %d", packet.command, packet.type, packet.option, packet.length);
    }

    list_return = serial.parsing(receive);

    ROS_INFO("Return");
    for (std::vector<information_packet_t>::iterator list_iter = list_return.begin(); list_iter != list_return.end(); ++list_iter) {
        information_packet_t packet = (*list_iter);
        ROS_INFO("Cmd: %d - Type: %c - Option: %c - lng: %d", packet.command, packet.type, packet.option, packet.length);
    }

    ROSController controller("test", nh, &serial);

    ros::spin();
}