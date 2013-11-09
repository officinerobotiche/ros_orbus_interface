#include <ros/ros.h>
#include <ros/callback_queue.h>

//messages
#include <std_msgs/String.h>

//services
#include <std_srvs/Empty.h>

#include "async_serial/ParserPacket.h"

using namespace std;

class ThreadTest {
public:

    ~ThreadTest() {
    }

public:

    ros::Subscriber sub_msg;
    ros::ServiceServer sub_srv;

    void msgCB(const std_msgs::String::ConstPtr& msg) {

        cout << "Message received" << endl;
        ros::Rate r(10);
        r.sleep();
        return;
    }

    bool srvCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

        cout << "Service invoked" << endl;
        ros::Rate r(0.2);
        ros::Time begin = ros::Time::now();
        r.sleep();
        ros::Duration sleeptime = ros::Time::now() - begin;
        cout << "Service finished " << "sleeping time " << sleeptime.toNSec() << endl;
        return true;
    }

    ThreadTest(ros::NodeHandle& nh1, ros::NodeHandle& nh2) {

        // first node handle for messages
        sub_msg = nh1.subscribe("chatter", 10, &ThreadTest::msgCB, this);

        //use second node handle for services
        sub_srv = nh2.advertiseService("chatter_service", &ThreadTest::srvCB, this);
    }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "threadtest");

    ros::NodeHandle nh1;

    ROS_INFO("Start");

    ParserPacket serial("/dev/ttyUSB0", 115200);
    
    sleep(2);

    ROS_INFO("Running");

    packet_t packet_send;
    packet_send.length = 0;

    serial.addPacket(&packet_send, VELOCITY, REQUEST, NULL);
    serial.addPacket(&packet_send, VELOCITY_MIS, REQUEST, NULL);

    string data_send(reinterpret_cast<const char*> (packet_send.buffer), packet_send.length);

    ROS_INFO("Send data: %s - length: %d", data_send.c_str(), packet_send.length);

    for (int i = 0; i < 10; ++i)
        try {
            packet_t packet;
            packet = serial.sendSyncPacket(packet_send, 3, boost::posix_time::millisec(200));
            string data_return(reinterpret_cast<const char*> (packet.buffer), packet.length);
            ROS_INFO("n: %d - Receive data %s - length: %d", i, data_return.c_str(), packet.length);

            list<information_packet_t> list_data = serial.parsing(packet);
            for (std::list<information_packet_t>::iterator list_iter = list_data.begin(); list_iter != list_data.end(); list_iter++) {
                information_packet_t packet = (*list_iter);
                ROS_INFO("Command: %c - Option: %c", packet.command, packet.option);
            }
        } catch (exception& e) {
            ROS_ERROR("%s", e.what());
        }

    ROS_INFO("Close");

    serial.close();

    //    //second nodehandle and service queue for working in second thread
    //    ros::NodeHandle nh2(nh1);
    //    ros::CallbackQueue service_queue(true);
    //    nh2.setCallbackQueue(&service_queue);
    //
    //    std::string simulation_string;
    //
    //    //pass both nodehandles
    //    ThreadTest test(nh1, nh2);
    //    ros::Rate r(10);
    //
    //    //start threads with different callback queue: the first one handles the global one
    //    ros::AsyncSpinner spinner(1);
    //    spinner.start();
    //
    //    ros::AsyncSpinner spinner2(1, &service_queue);
    //    spinner2.start();
    //
    //    ros::waitForShutdown();
}