// ArRobot imports
#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
#include <Aria.h>
#include <ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#else
#include <Aria/Aria.h>
#include <Aria/ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#endif
#include <sstream>

// ROS2 Imports
#include "rclcpp/rclcpp.hpp"

class Ros2AriaNode : public rclcpp ::Node
{
public:
    Ros2AriaNode() : Node("ros2_aria_node")
    {
        // Initialising pointers


        // Publishers

        // Subscribers

        // Servers

    }

    ~Ros2AriaNode()
    {
        // disable motors and sonar.
        robot->disableMotors();
        robot->disableSonar();

        robot->stopRunning();
        robot->waitForRunExit();
        Aria::shutdown();
    }

    int Setup()
    {
        robot = new ArRobot();
        ArArgumentBuilder *args = new ArArgumentBuilder();        // never freed
        ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
        argparser->loadDefaultArguments();                        // adds any arguments given in /etc/Aria.args. Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
        // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
        // for wireless serial connection. Otherwise, interpret it as a serial port name.
        size_t colon_pos = serial_port.find(":");
        if (colon_pos != std::string::npos)
        {
            args->add("-remoteHost");
            args->add(serial_port.substr(0, colon_pos).c_str());
            args->add("-remoteRobotTcpPort");
            args->add(serial_port.substr(colon_pos + 1).c_str());
        }
        else
        {
            args->add("-robotPort %s", serial_port.c_str());
        }

        // if a baud rate was specified in baud parameter
        if (serial_baud != 0)
        {
            args->add("-robotBaud %d", serial_baud);
        }

        // Connect to the robot
        conn = new ArRobotConnector(argparser, robot); // warning never freed
        if (!conn->connectRobot())
        {
            RCLCPP_ERROR(this->get_logger(), "Ros2Aria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
            return 1;
        }

        // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
        if (!Aria::parseArgs())
        {
            RCLCPP_ERROR(this->get_logger(), "Ros2Aria: ARIA error parsing ARIA startup parameters!");
            return 1;
        }

        trans_accel = robot->getTransAccel() / 1000;
        trans_decel = robot->getTransDecel() / 1000;
        lat_accel = robot->getLatAccel() / 1000;
        lat_decel = robot->getLatDecel() / 1000;
        rot_accel = robot->getRotAccel() * M_PI / 180;
        rot_decel = robot->getRotDecel() * M_PI / 180;

        TicksMM = 0;
        DriftFactor = -99999;
        RevCount = 0;

        // Enable the motors
        robot->enableMotors();

        // disable sonars on startup
        robot->disableSonar();

        // callback will be called by ArRobot background processing thread for every SIP data packet received from robot
        robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

        // Run ArRobot background processing thread
        robot->runAsync(true);

        RCLCPP_INFO(this->get_logger(), "ros2aria: Setup complete");
        return 0;
    }

protected:
    // Aria library dependent variables
    ArRobotConnector *conn;
    // ArLaserConnector *laserConnector;
    ArRobot *robot;
    // ros2aria::msg::BumperState bumpers;
    ArPose pos;
    ArFunctorC<Ros2AriaNode> myPublishCB;
    // ArRobot::ChargeState batteryCharge;

    // Robot Calibration Parameters
    int serial_baud = 0;
    std::string serial_port = "/dev/ttyUSB0";
    int TicksMM, DriftFactor, RevCount;
    float trans_accel, trans_decel, lat_accel, lat_decel, rot_accel, rot_decel;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    Aria::init();

    auto node = std::make_shared<Ros2AriaNode>();

    if (node->Setup() != 0)
    {
        RCLCPP_FATAL(node->get_logger(), "Ros2Aria: ROS2 Node Setup Failed !!!");
        return -1;
    }

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Ros2Aria: Exiting !!!");

    return 0;
}