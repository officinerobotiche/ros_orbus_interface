/* 
 * File:   ROSSensorController.cpp
 * Author: raffaello
 * 
 * Created on 25 October 2013, 13:27
 */

#include "serial_controller/ROSSensorController.h"

ROSSensorController::ROSSensorController(std::string name_node, const ros::NodeHandle& nh, Serial* serial, ServiceSerial* service_serial)
: nh_(nh) {
    name_node_ = name_node; // Initialize node name
    this->serial_ = serial; // Initialize serial port
    this->service_serial_ = service_serial; //Initialize service with serial
    serial_->asyncPacket(&ROSSensorController::actionAsync, this);

    //Open Publisher
    pub_laser_sharp_ = nh_.advertise<sensor_msgs::LaserScan>("/" + name_node + "/" + laser_sharp_string, NUMBER_PUBLISHER,
            boost::bind(&ROSSensorController::connectCallback, this, _1));

    //Open Subscriber

    //Open Service

    //TODO
    test();
}

ROSSensorController::ROSSensorController(const ROSSensorController& orig) {
}

ROSSensorController::~ROSSensorController() {
}

void ROSSensorController::loadParameter() {

}

void ROSSensorController::actionAsync(packet_t packet) {
    ROS_INFO("ROS Sensor Controller Async");
}

void ROSSensorController::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s", pub.getSubscriberName().c_str());
}

void ROSSensorController::transformPoint(const tf::TransformListener& listener) {
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "base_laser";

    //we'll just use the most recent transform available for our simple example
    laser_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    laser_point.point.x = 1.0;
    laser_point.point.y = 0.2;
    laser_point.point.z = 0.0;

    try {
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", laser_point, base_point);

        ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                laser_point.point.x, laser_point.point.y, laser_point.point.z,
                base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    }
}

void ROSSensorController::test() {
    unsigned int num_readings = 100;
    double laser_frequency = 40;
    double ranges[num_readings];
    double intensities[num_readings];

    int count = 0;
    ros::Rate r(1.0);
    while (nh_.ok()) {
        //generate some fake data for our laser scan
        for (unsigned int i = 0; i < num_readings; ++i) {
            ranges[i] = count;
            intensities[i] = 100 + count;
        }
        ros::Time scan_time = ros::Time::now();

        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "laser_frame";
        scan.angle_min = -1.57;
        scan.angle_max = 1.57;
        scan.angle_increment = 3.14 / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 100.0;

        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);
        for (unsigned int i = 0; i < num_readings; ++i) {
            scan.ranges[i] = ranges[i];
            scan.intensities[i] = intensities[i];
        }

        pub_laser_sharp_.publish(scan);

        ++count;
        r.sleep();
    }
}