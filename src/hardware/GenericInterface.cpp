#include "hardware/GenericInterface.h"

#include <regex>

namespace ORInterface
{

#define BIT_MASK(x)                       (1 << (x))
#define REGISTER_MASK_READ(reg, mask)     ((*(reg) & (mask)) == (mask))

GenericInterface::GenericInterface(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, orbus::serial_controller *serial)
    : DiagnosticTask("board")
    , mNh(nh)
    , private_mNh(private_nh)
    , mSerial(serial)
    , serial_status(true)
    , code_date("Unknown"), code_version("Unknown"), code_author("Unknown"), code_board_type("Unknown"), code_board_name("Unknown")
{
    bool initsystemCallback = mSerial->addCallback(&GenericInterface::systemFrame, this, HASHMAP_SYSTEM);

    bool initgpioCallback = mSerial->addCallback(&GenericInterface::peripheralFrame, this, HASHMAP_PERIPHERALS);

    //Publisher
    pub_time = private_mNh.advertise<orbus_interface::BoardTime>("system", 10,
                boost::bind(&GenericInterface::connectCallback, this, _1));

    pub_peripheral = private_mNh.advertise<orbus_interface::Peripheral>("peripheral", 10,
                boost::bind(&GenericInterface::connectCallback, this, _1));
    //Subscriber
    sub_peripheral = private_mNh.subscribe("cmd_peripheral", 1, &GenericInterface::gpio_subscriber_Callback, this);

    //Services
    srv_board = private_mNh.advertiseService("system", &GenericInterface::service_Callback, this);

    // GPIO
    srv_gpio = private_mNh.advertiseService("gpio", &GenericInterface::gpio_Callback, this);

    // Build a packet
    packet_information_t frame_code_date = CREATE_PACKET_RESPONSE(SYSTEM_CODE_DATE, HASHMAP_SYSTEM, PACKET_REQUEST);
    packet_information_t frame_code_version = CREATE_PACKET_RESPONSE(SYSTEM_CODE_VERSION, HASHMAP_SYSTEM, PACKET_REQUEST);
    packet_information_t frame_code_author = CREATE_PACKET_RESPONSE(SYSTEM_CODE_AUTHOR, HASHMAP_SYSTEM, PACKET_REQUEST);
    packet_information_t frame_code_board_type = CREATE_PACKET_RESPONSE(SYSTEM_CODE_BOARD_TYPE, HASHMAP_SYSTEM, PACKET_REQUEST);
    packet_information_t frame_code_board_name = CREATE_PACKET_RESPONSE(SYSTEM_CODE_BOARD_NAME, HASHMAP_SYSTEM, PACKET_REQUEST);

    if(mSerial->addFrame(frame_code_date)->addFrame(frame_code_version)->addFrame(frame_code_author)->addFrame(frame_code_board_type)->addFrame(frame_code_board_name)->sendList())
    {
        ROS_DEBUG_STREAM("Send Service information messages");
    }
    else
    {
        ROS_ERROR_STREAM("Any messages from board");
    }

    // Initialize all GPIO
    initGPIO();
}

void GenericInterface::initGPIO()
{
    std::vector<int> gpio_list;
    if(private_mNh.hasParam("gpio"))
    {
        private_mNh.getParam("gpio", gpio_list);
        setupGPIO(gpio_list);
    }
}

void GenericInterface::setupGPIO(std::vector<int> gpio_list)
{
    peripheral_gpio_map_t gpio;
    gpio.bitset.command = PERIPHERALS_GPIO_SET;
    gpio.bitset.port = 1;

    peripherals_gpio_port_t input, output;
    input.port = 0;
    output.port = 0;
    unsigned int counter_input = 0, counter_output = 0;

    for(unsigned i=0; i < gpio_list.size(); ++i)
    {
        switch(gpio_list.at(i))
        {
        case PERIPHERAL_GPIO_OUTPUT:
            output.port += BIT_MASK(i);
            counter_output++;
            break;
        case PERIPHERAL_GPIO_INPUT:
            input.port += BIT_MASK(i);
            counter_input++;
            break;
        default:
            input.port += BIT_MASK(i);
            counter_input++;
            break;
        }

    }
    ROS_INFO_STREAM("Port config INPUT n=" << counter_input << " - OUTPUT n=" << counter_output);
    // Build a packet
    message_abstract_u temp_input;
    temp_input.gpio.set.port.len = input.len;
    temp_input.gpio.set.port.port = input.port;
    temp_input.gpio.set.type = PERIPHERAL_GPIO_INPUT;
    packet_information_t frame_input = CREATE_PACKET_DATA(gpio.message, HASHMAP_PERIPHERALS, temp_input);

    message_abstract_u temp_output;
    temp_output.gpio.set.port.len = output.len;
    temp_output.gpio.set.port.port = output.port;
    temp_output.gpio.set.type = PERIPHERAL_GPIO_OUTPUT;
    packet_information_t frame_output = CREATE_PACKET_DATA(gpio.message, HASHMAP_PERIPHERALS, temp_output);

    // Update and send list configuration GPIO
    mSerial->addFrame(frame_input)->addFrame(frame_output)->sendList();
}

void GenericInterface::initializeDiagnostic()
{

    ROS_INFO_STREAM("Name board: " << code_board_name << " - " << code_version);
    diagnostic_updater.setHardwareID(code_board_name);

    // Initialize this diagnostic interface
    diagnostic_updater.add(*this);
}

void GenericInterface::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    ROS_DEBUG_STREAM("DIAGNOSTIC Generic interface I'm here!");
    // Build a packet
    packet_information_t frame = CREATE_PACKET_RESPONSE(SYSTEM_TIME, HASHMAP_SYSTEM, PACKET_REQUEST);

    // Build a packet
    gpio_map.bitset.port = 1;
    gpio_map.bitset.command = PERIPHERALS_GPIO_DIGITAL;
    packet_information_t frame_gpio = CREATE_PACKET_RESPONSE(gpio_map.message, HASHMAP_PERIPHERALS, PACKET_REQUEST);

    // Add packet in the frame
    if(mSerial->addFrame(frame)->addFrame(frame_gpio)->sendList())
    {
        ROS_DEBUG_STREAM("Request Diagnostic COMPLETED");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to receive packet from uNav");
    }

    stat.add("Name board", code_board_name);
    stat.add("Type board", code_board_type);
    stat.add("Author", code_author);
    stat.add("Version", code_version);
    stat.add("Build", code_date);

    stat.add("Idle (%)", (int) msg_system.idle);
    stat.add("ADC (nS)", (int) msg_system.ADC);
    stat.add("LED (nS)", (int) msg_system.led);
    stat.add("Serial parser (nS)", (int) msg_system.serial_parser);
    stat.add("I2C (nS)", (int) msg_system.I2C);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Board ready!");
}

void GenericInterface::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s - %s", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
}

void GenericInterface::convertGPIO(peripherals_gpio_port_t data) {
    //ROS_INFO_STREAM("Port len: " << (int) data.len << " data: " << data.port);
    msg_peripheral.gpio.clear();
    for(unsigned i=0; i < data.len; ++i)
    {
        int n = BIT_MASK(i);
        msg_peripheral.gpio.push_back(REGISTER_MASK_READ(&data.port, n));
    }
}

void GenericInterface::peripheralFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    ROS_DEBUG_STREAM("Frame [Option: " << option << ", HashMap: " << type << ", Command: " << (int) command << "]");
    peripheral_gpio_map_t peripheral;
    peripheral.message = command;
    switch(peripheral.bitset.command)
    {
    case PERIPHERALS_GPIO_DIGITAL:
        convertGPIO(message.gpio.port);
        // publish a message
        msg_peripheral.header.stamp = ros::Time::now();
        pub_peripheral.publish(msg_peripheral);
        break;
    case PERIPHERALS_GPIO_SET:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("GPIO SET Data");
        }
        break;
    default:
        ROS_ERROR_STREAM("Peripheral message \""<< command << "\"=(" << (int) command << ")" << " does not implemented!");
        break;
    }
}

void GenericInterface::systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    ROS_DEBUG_STREAM("Frame [Option: " << option << ", HashMap: " << type << ", Command: " << command << "]");
    switch (command) {
    case SYSTEM_CODE_DATE:
        code_date = string((char*)message.system.service);
        break;
    case SYSTEM_CODE_VERSION:
        code_version = string((char*)message.system.service);
        break;
    case SYSTEM_CODE_AUTHOR:
        code_author = string((char*)message.system.service);
        break;
    case SYSTEM_CODE_BOARD_TYPE:
        code_board_type = string((char*)message.system.service);
        break;
    case SYSTEM_CODE_BOARD_NAME:
        code_board_name = string((char*)message.system.service);
        break;
    case SYSTEM_TIME:
        msg_system.idle = message.system.time.idle;
        msg_system.ADC = message.system.time.adc;
        msg_system.led = message.system.time.led;
        msg_system.serial_parser = message.system.time.parser;
        msg_system.I2C = message.system.time.i2c;
        // publish a message
        msg_system.header.stamp = ros::Time::now();
        pub_time.publish(msg_system);
        break;
    default:
        ROS_ERROR_STREAM("System message \""<< command << "\"=(" << (int) command << ")" << " does not implemented!");
        break;
    }
}

void GenericInterface::gpio_subscriber_Callback(const orbus_interface::Peripheral::ConstPtr& msg)
{
    peripherals_gpio_port_t port;
    port.port = 0;
    port.len = (msg->gpio.size() > 16 ? 16 : msg->gpio.size());
    for(unsigned i=0; i < port.len; ++i)
    {
        if(msg->gpio.at(i) > 0)
        {
            port.port += BIT_MASK(i);
        }
    }
    //ROS_INFO_STREAM("Gpio: " << port.port);

    peripheral_gpio_map_t gpio;
    gpio.bitset.command = PERIPHERALS_GPIO_DIGITAL;
    gpio.bitset.port = 1;
    // Build a packet
    message_abstract_u temp;
    temp.gpio.port.len = port.len;
    temp.gpio.port.port = port.port;
    packet_information_t frame = CREATE_PACKET_DATA(gpio.message, HASHMAP_PERIPHERALS, temp);
    // Send new configuration
    mSerial->addFrame(frame)->sendList();
}

int GenericInterface::binary_decimal(int n) /* Function to convert binary to decimal.*/
{
    int decimal=0, i=0, rem;
    while (n!=0)
    {
        rem = n%10;
        n/=10;
        decimal += rem*pow(2,i);
        ++i;
    }
    return decimal;
}

bool GenericInterface::gpio_Callback(orbus_interface::GPIO::Request &req, orbus_interface::GPIO::Response &msg) {

    if(req.gpio.size() > 0)
    {
        std::vector<int> gpio_list;
        vector<unsigned char> data = req.gpio;
        for(unsigned i=0; i < data.size(); ++i)
        {
            int number = (int) data.at(i);
            //ROS_INFO_STREAM("Data: " << number);
            gpio_list.push_back(number);
        }
        setupGPIO(gpio_list);
        msg.information = "Setup ok!";
    }
    else
    {
        msg.information = "\nUsage: [X,X,...,X]\n"
                          "PORT is the input value for your port\n"
                          "0 for OUTOUT level\n"
                          "1 for INPUT level\n"
                          "Example: [1,0,1,1,1]";
    }

    return true;
}

bool GenericInterface::service_Callback(orbus_interface::Service::Request &req, orbus_interface::Service::Response &msg) {
    // Convert to lower case
    std::transform(req.service.begin(), req.service.end(), req.service.begin(), ::tolower);
    //ROS_INFO_STREAM("Name request: " << req.service);
    if(req.service.compare("info") == 0)
    {
        msg.information = "\nName board: " + code_board_name + "\n"
                          "Board type: " + code_board_type + "\n"
                          "Author: " + code_author + "\n"
                          "Version: " + code_version + "\n"
                          "Build: " + code_date + "\n";
    }
    else if(req.service.compare("reset") == 0)
    {
        packet_information_t frame_reset = CREATE_PACKET_RESPONSE(SYSTEM_RESET, HASHMAP_SYSTEM, PACKET_REQUEST);
        // Send reset
        mSerial->addFrame(frame_reset)->sendList();
        // return message
        msg.information = "System reset";
    }
    else
    {
        msg.information = "\nList of commands availabes: \n"
                          "* info  - information about this board \n"
                          "* reset - " + code_board_name + " board software reset\n"
                          "* help  - this help.";
    }
    return true;
}

}
