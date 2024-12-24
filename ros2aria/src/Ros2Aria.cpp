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
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
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
  void cmdvel_watchdog();

protected:
  // Class variables
  std::string serial_port;
  int serial_baud;
  std::string frame_id_odom;
  std::string frame_id_base_link;
  std::string frame_id_bumper;
  std::string frame_id_sonar;
  std_msgs::msg::Bool motors_state;
  std_msgs::msg::Bool continuous;
  bool published_motors_state;
  std_msgs::msg::Int8 recharge_state;
  std_msgs::msg::Float64 batteryVoltage;
  nav_msgs::msg::Odometry position;
  geometry_msgs::msg::TransformStamped odom_trans;

  // ArRobot library variables
  ArRobotConnector *conn;
  ArRobot *robot;
  ArPose pos;
  ArFunctorC<Ros2AriaNode> myPublishCB;

  // Pointers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr recharge_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_volate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_of_charge_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;

  tf2_ros::TransformBroadcaster odom_broadcaster_;

  rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer_;
  rclcpp::Time veltime;
};

Ros2AriaNode::Ros2AriaNode()
    : Node("ros2aria_run"),
      serial_port("/dev/ttyUSB0"),
      serial_baud(0),
      conn(nullptr),
      robot(nullptr),
      myPublishCB(this, &Ros2AriaNode::publish),
      odom_broadcaster_(this)
{
  // Declare parameters
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("serial_baud", 0);
  this->declare_parameter("frame_id_odom", "odom");
  this->declare_parameter("frame_id_base_link", "base_link");
  this->declare_parameter("frame_id_bumper", "bumpers");
  this->declare_parameter("frame_id_sonar", "sonar");

  // Get parameters
  serial_port = this->get_parameter("serial_port").as_string();
  serial_baud = this->get_parameter("serial_baud").as_int();
  frame_id_odom = this->get_parameter("frame_id_odom").as_string();
  frame_id_base_link = this->get_parameter("frame_id_base_link").as_string();
  frame_id_bumper = this->get_parameter("frame_id_bumper").as_string();
  frame_id_sonar = this->get_parameter("frame_id_sonar").as_string();

  // Publishers
  // Motors QoS and Motor Publisher with required settings
  rclcpp::QoS common_qos(5);
  common_qos.transient_local();
  motors_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("motors_state", common_qos);
  motors_state.data = false;
  published_motors_state = false;
  recharge_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("recharge_state", common_qos);
  recharge_state.data = -2;
  battery_volate_pub_ = this->create_publisher<std_msgs::msg::Float64>("battery_state_of_charge", rclcpp::QoS(1000));
  pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pose", rclcpp::QoS(1000));

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

  // Watchdog timer for cmd_vel timeout if the robot is not operating in continuous mode
  std::chrono::duration<int, std::nano> cmdvel_timeout = std::chrono::duration<int, std::nano>(600000000); // 0.6 seconds
  if (cmdvel_timeout.count() > 0 && (!continuous.data))
  {
    cmdvel_watchdog_timer_ = this->create_wall_timer(
        cmdvel_timeout,
        std::bind(&Ros2AriaNode::cmdvel_watchdog, this));
  }

  RCLCPP_INFO(this->get_logger(), "Ros2Aria : Setup complete");
  return 0;
}

void Ros2AriaNode::publish()
{

  // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
  // Publish Odometry
  pos = robot->getPose();
  geometry_msgs::msg::Pose ros2_pose;
  tf2::Quaternion ros2_quat;
  ros2_quat.setRPY(0, 0, pos.getTh() * M_PI / 180);
  tf2::Vector3 ros2_vec(pos.getX() / 1000.0, pos.getY() / 1000.0, 0.0);
  tf2::Transform ros2_tf(ros2_quat, ros2_vec);
  tf2::toMsg(ros2_tf, ros2_pose);
  position.twist.twist.linear.x = robot->getVel() / 1000.0; // Aria returns velocity in mm/s.
  position.twist.twist.linear.y = robot->getLatVel() / 1000.0;
  position.twist.twist.angular.z = robot->getRotVel() * M_PI / 180;
  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = this->now();
  pose_pub_->publish(position);
  RCLCPP_DEBUG(this->get_logger(), "Ros2Aria : publish: (time %f) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f", 
    position.header.stamp.sec + position.header.stamp.nanosec * 1e-9, 
    (double)position.pose.pose.position.x,
    (double)position.pose.pose.position.y,
    (double)position.pose.pose.orientation.w,
    (double)position.twist.twist.linear.x,
    (double)position.twist.twist.linear.y,
    (double)position.twist.twist.angular.z
  );

  // publishing transform odom->base_link
  odom_trans.header.stamp = this->now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;
  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, pos.getTh() * M_PI / 180);
  odom_trans.transform.rotation = tf2::toMsg(quat);
  odom_broadcaster_.sendTransform(odom_trans);

  // Publish live battery voltage
  batteryVoltage.data = robot->getRealBatteryVoltageNow(); // Use getBatteryVoltageNow() if you want normalized value from 0 to 12V.
  battery_volate_pub_->publish(batteryVoltage);

  // Publish the state of charge if charing
  if (robot->haveStateOfCharge())
  {
    std_msgs::msg::Float32 soc;
    soc.data = robot->getStateOfCharge() / 100.0;
    state_of_charge_pub_->publish(soc);
  }

  // Publish recharge state on change
  char current_recharge_state = robot->getChargeState();
  if (current_recharge_state != recharge_state.data)
  {
    recharge_state.data = current_recharge_state;
    recharge_state_pub_->publish(recharge_state);
    RCLCPP_INFO(this->get_logger(), "Ros2Aria : Publishing New Recharge State %d.", current_recharge_state);
  }

  // Publish motor state on change
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

void Ros2AriaNode::cmdvel_watchdog()
{
  // Stop the robot if it does not receive any new velocity commands for 0.6 seconds
  if (this->now() - veltime > rclcpp::Duration(0, 600000000))
  {
    robot->lock();
    robot->setVel(0.0);
    if (robot->hasLatVel())
      robot->setLatVel(0.0);
    robot->setRotVel(0.0);
    robot->unlock();
  }
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