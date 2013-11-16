
#include <ros/ros.h>
#include "async_serial/ParserPacket.h"
#include "serial_controller/ROSController.h"
#include "serial_controller/ROSMotionController.h"

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "threadtest");

    ros::NodeHandle nh;

    ROS_INFO("Start");

    ParserPacket serial("/dev/ttyUSB0", 115200);

    string name_node = "test";

    if (nh.hasParam(name_node + "/name_board")) {
        //TODO
        string param_name_board;
        nh.getParam(name_node + "/name_board", param_name_board);
        if (param_name_board.compare("Motion Control") == 0) {
            ROSMotionController controller(name_node, nh, &serial);
            controller.loadParameter();
        }
    } else {
        ROSController controller(name_node, nh, &serial);
        controller.loadParameter();
    }

    ros::spin();

    serial.close();
}