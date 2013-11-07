#include <ros/ros.h>
#include <ros/callback_queue.h>

//messages
#include <std_msgs/String.h>

//services
#include <std_srvs/Empty.h>

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