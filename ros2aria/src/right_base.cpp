#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
  #include <ArRobotConfigPacketReader.h>
#else
  #include <Aria/Aria.h>
  #include <Aria/ArRobotConfigPacketReader.h>
#endif
#include "rclcpp/rclcpp.hpp"
#include <sstream>

class RosAriaNode : public rclcpp::Node
{
public:
  RosAriaNode();
  virtual ~RosAriaNode();
    
public:
  int Setup();
  void publish();

protected:
  std::string serial_port;
  int serial_baud;

  ArRobotConnector *conn;
  ArRobot *robot;
  ArFunctorC<RosAriaNode> myPublishCB;
};

RosAriaNode::RosAriaNode()
  : Node("ros2aria_run"),
    serial_port("/dev/ttyUSB0"),
    serial_baud(0),
    conn(nullptr),
    robot(nullptr),
    myPublishCB(this, &RosAriaNode::publish)
{
  // Declare parameters
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("serial_baud", 0);

  // Get parameters
  serial_port = this->get_parameter("serial_port").as_string();
  serial_baud = this->get_parameter("serial_baud").as_int();
}

RosAriaNode::~RosAriaNode()
{
  if (robot) {
    // disable motors and sonar.
    robot->disableMotors();
    robot->disableSonar();

    robot->stopRunning();
    robot->waitForRunExit();
  }
  Aria::shutdown();
}

int RosAriaNode::Setup()
{
  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments();

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-remoteHost");
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort");
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-robotPort %s", serial_port.c_str());
  }

  if(serial_baud != 0)
  {
    args->add("-robotBaud %d", serial_baud);
  }

  conn = new ArRobotConnector(argparser, robot);
  if (!conn->connectRobot()) {
    RCLCPP_ERROR(this->get_logger(), "RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
    return 1;
  }

  if(!Aria::parseArgs())
  {
    RCLCPP_ERROR(this->get_logger(), "RosAria: ARIA error parsing ARIA startup parameters!");
    return 1;
  }

  robot->enableMotors();
  robot->disableSonar();
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);
  robot->runAsync(true);

  RCLCPP_INFO(this->get_logger(), "rosaria: Setup complete");
  return 0;
}

void RosAriaNode::publish()
{
  RCLCPP_INFO(this->get_logger(), "publishing");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  Aria::init();

  auto node = std::make_shared<RosAriaNode>();

  if(node->Setup() != 0)
  {
    RCLCPP_ERROR(node->get_logger(), "RosAria: ROS node setup failed...");
    return -1;
  }

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}