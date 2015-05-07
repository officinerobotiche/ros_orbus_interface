#ifndef ORB_HARDWARE_H
#define ORB_HARDWARE_H

#include <ros/ros.h>
//#include <ros_serial_bridge/Service.h>
//#include <ros_serial_bridge/Process.h>
//#include <ros_serial_bridge/Update.h>
#include <std_srvs/Empty.h>
#include "../async_serial/ParserPacket.h"
#include "hardware_interface/robot_hw.h"

const std::string measure_string = "measure";
const std::string left_string = "Left";
const std::string right_string = "Right";
const std::string all_string = "all";
const std::string tf_string = "tf";

/**
 * Thrown if timeout occurs
 */
class controller_exception : public std::runtime_error {
public:

    controller_exception(const std::string& arg) : runtime_error(arg) {
    }
};

class ORBHardware : public hardware_interface::RobotHW {
public:
    ORBHardware(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, ParserPacket* serial);

    void updateDiagnostics();

    void reportLoopDuration(const ros::Duration &duration);

    virtual ~ORBHardware();

    void loadParameter();
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    std::string getNameBoard();
    std::string getTypeBoard();

    void addVectorPacketRequest(const boost::function<void (std::vector<packet_information_t>*) >& callback);

    template <class T> void addVectorPacketRequest(void(T::*fp)(std::vector<packet_information_t>*), T* obj) {
        addVectorPacketRequest(boost::bind(fp, obj, _1));
    }
    void clearVectorPacketRequest();

    void addParameterPacketRequest(const boost::function<void (std::vector<packet_information_t>*) >& callback);

    template <class T> void addParameterPacketRequest(void(T::*fp)(std::vector<packet_information_t>*), T* obj) {
        addParameterPacketRequest(boost::bind(fp, obj, _1));
    }
    void clearParameterPacketRequest();

    void addTimerEvent(const boost::function<void (const ros::TimerEvent&) >& callback);

    template <class T> void addTimerEvent(void(T::*fp)(const ros::TimerEvent&), T* obj) {
        addTimerEvent(boost::bind(fp, obj, _1));
    }
    void clearTimerEvent();

protected:
    //Initialization object
    ros::NodeHandle nh_; //NameSpace for bridge controller
    ros::NodeHandle private_nh_; //Private NameSpace for bridge controller
    ParserPacket* serial_; //Serial object to comunicate with PIC device
    std::string name_board_, version_, name_author_, compiled_, type_board_;
private:

    typedef boost::function<void (std::vector<packet_information_t>*) > callback_add_packet_t;
    typedef boost::function<void (const ros::TimerEvent&) > callback_timer_event_t;
    typedef boost::function<bool (const ros::TimerEvent&, std::vector<packet_information_t>*) > callback_add_event_t;
    callback_add_packet_t callback_add_packet, callback_add_parameter;
    callback_add_event_t callback_alive_event;
    callback_timer_event_t callback_timer_event;

    ros::ServiceServer srv_board, srv_process;
    ros::Publisher pub_time_process;

    //ros_serial_bridge::Process time_process;
    double step_timer, tm_mill, k_time;
    int number_process;
    bool init_number_process;
    std::map<std::string, int> map_error_serial;

    std::vector<packet_information_t> updatePacket();

    float getTimeProcess(float process_time);
    void errorPacket(const unsigned char& command, const message_abstract_u* packet);
    void defaultPacket(const unsigned char& command, const message_abstract_u* packet);

    std::string getNameError(int number);
    std::string getBoardSerialError();
    packet_information_t encodeNameProcess(int number);
    void requestNameProcess();
    packet_information_t encodeServices(char command, unsigned char* buffer = NULL, size_t len = 0);
    void resetBoard(unsigned int repeat = 3);
    void decodeServices(const char command, const unsigned char* buffer);

    //process_t get_process(std::string name);

    //bool processServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
    //bool service_Callback(ros_serial_bridge::Service::Request &req, ros_serial_bridge::Service::Response &msg);
};

#endif // ORB_HARDWARE_H
