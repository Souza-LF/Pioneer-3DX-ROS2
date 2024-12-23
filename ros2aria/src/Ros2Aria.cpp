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
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sstream>

using std::placeholders::_1;

class Ros2AriaNode : public rclcpp::Node
{
public:
  Ros2AriaNode();
  virtual ~Ros2AriaNode();

public:
  int Setup();
  void publish();
  void cmdvel_cb(geometry_msgs::msg::Twist::ConstSharedPtr);

protected:
  // Class variables
  std::string serial_port;
  int serial_baud;
  std_msgs::msg::Bool motors_state;
  bool published_motors_state;

  // ArRobot library variables
  ArRobotConnector *conn;
  ArRobot *robot;
  ArFunctorC<Ros2AriaNode> myPublishCB;

  // Pointers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;

  rclcpp::Time veltime;
};

Ros2AriaNode::Ros2AriaNode()
    : Node("ros2aria_run"),
      serial_port("/dev/ttyUSB0"),
      serial_baud(0),
      conn(nullptr),
      robot(nullptr),
      myPublishCB(this, &Ros2AriaNode::publish)
{
  // Declare parameters
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("serial_baud", 0);

  // Get parameters
  serial_port = this->get_parameter("serial_port").as_string();
  serial_baud = this->get_parameter("serial_baud").as_int();

  // Publishers
  // Motors QoS and Motor Publisher with required settings
  rclcpp::QoS motor_qos(5);
  motor_qos.transient_local();
  motors_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("motors_state", motor_qos);
  motors_state.data = false;
  published_motors_state = false;

  veltime = this->now();
}

Ros2AriaNode::~Ros2AriaNode()
{
  if (robot)
  {
    // disable motors and sonar.
    robot->disableMotors();
    robot->disableSonar();

    robot->stopRunning();
    robot->waitForRunExit();
  }
  Aria::shutdown();
}

int Ros2AriaNode::Setup()
{
  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder();        //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments();

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
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

  if (serial_baud != 0)
  {
    args->add("-robotBaud %d", serial_baud);
  }

  conn = new ArRobotConnector(argparser, robot);
  if (!conn->connectRobot())
  {
    RCLCPP_ERROR(this->get_logger(), "Ros2Aria : ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
    return 1;
  }

  if (!Aria::parseArgs())
  {
    RCLCPP_ERROR(this->get_logger(), "Ros2Aria : ARIA error parsing ARIA startup parameters!");
    return 1;
  }

  robot->enableMotors();
  robot->disableSonar();
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);
  robot->runAsync(true);

  // Subcriber to cmd_vel topic
  cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&Ros2AriaNode::cmdvel_cb, this, _1));

  RCLCPP_INFO(this->get_logger(), "Ros2Aria : Setup complete");
  return 0;
}

void Ros2AriaNode::publish()
{
  bool current_motor_state = robot->areMotorsEnabled();
  if (current_motor_state != motors_state.data || !published_motors_state)
  {
    motors_state.data = current_motor_state;
    motors_state_pub_->publish(motors_state);
    RCLCPP_INFO(this->get_logger(), "Ros2Aria : Publishing New Motor State %d.", current_motor_state);
    published_motors_state = true;
  }
  // RCLCPP_INFO(this->get_logger(), "Callback invoked");
}

void Ros2AriaNode::cmdvel_cb(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  veltime = this->now();
  RCLCPP_INFO(this->get_logger(), "New speed: [%0.2f, %0.2f](%0.3f)", msg->linear.x * 1e3, msg->angular.z, veltime.seconds());
  robot->lock();
  robot->setVel(msg->linear.x * 1e3);
  if (robot->hasLatVel())
    robot->setLatVel(msg->linear.y * 1e3);
  robot->setRotVel(msg->angular.z * 180 / M_PI);
  robot->unlock();
  RCLCPP_DEBUG(this->get_logger(), "Ros2Aria : Velocity sent to Aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.seconds(), (double)msg->linear.x * 1e3, (double)msg->linear.y * 1e3, (double)msg->angular.z * 180 / M_PI);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Aria::init();

  auto node = std::make_shared<Ros2AriaNode>();

  if (node->Setup() != 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Ros2Aria : ROS node setup failed...");
    return -1;
  }

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}