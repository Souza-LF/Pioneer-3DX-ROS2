#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
#include <Aria.h>
#include <ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#else
#include <Aria/Aria.h>
#include <Aria/ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#endif
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/int8.hpp>

#include <sstream>

class Ros2AriaNode : public rclcpp::Node
{
public:
    Ros2AriaNode() : Node("ros2_aria_node"), cmd_vel_timeout(1, 0), odom_broadcaster(this), robot(NULL), conn(NULL), myPublishCB(this, &Ros2AriaNode::publish), TicksMM(-1), DriftFactor(-99999), RevCount(-1), debug_aria(false)
    {
        // Initialize publishers, subscribers, services, variables etc. here

        // Publishers
        pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pose", rclcpp::QoS(1000));
        // Create a QoS profile with durability set to TransientLocal for latching
        auto common_qos = rclcpp::QoS(rclcpp::KeepLast(5)).transient_local();
        motors_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("motors_state", common_qos);
        recharge_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("battery_recharge_state", common_qos);
        motors_state.data = false;
        published_motors_state = false;
        recharge_state.data = -2;

        // subscribe to command topics
        // cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        //     "cmd_vel", 10, std::bind(&Ros2AriaNode::cmdvel_cb, this, std::placeholders::_1));

        // Advertise enable/disable services
        // enable_srv = this->create_service<std_srvs::srv::Empty>("enable_motors", std::bind(&Ros2AriaNode::enable_motors_cb, this, std::placeholders::_1, std::placeholders::_2));
        // disable_srv = this->create_service<std_srvs::srv::Empty>("disable_motors", std::bind(&Ros2AriaNode::disable_motors_cb, this, std::placeholders::_1, std::placeholders::_2));

        veltime = this->now();
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

    int Setup();
    void cmdvel_cb(const geometry_msgs::msg::Twist &msg);
    void cmdvel_watchdog(); // No TimerEvent parameter
    void updateParameters();
    void spin();
    void publish();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub_;
    std_msgs::msg::Bool motors_state;
    bool published_motors_state;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr recharge_state_pub_;
    std_msgs::msg::Int8 recharge_state;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_srv;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disable_srv;
    bool enable_motors_cb(const std_srvs::srv::Empty::Request &request, const std_srvs::srv::Empty::Response &response);
    bool disable_motors_cb(const std_srvs::srv::Empty::Request &request, const std_srvs::srv::Empty::Response &response);

    rclcpp::Time veltime;
    rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer;
    rclcpp::Duration cmd_vel_timeout;

    int serial_baud = 0;
    std::string serial_port = "/dev/ttyUSB1";

    // Debug Aria
    bool debug_aria;
    std::string aria_log_filename;

    // For odom->base_link transform
    tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::msg::TransformStamped odom_trans;

    std::string frame_id_odom = "odom";
    std::string frame_id_base_link = "base_link";
    std::string frame_id_bumper = "bumpers";
    std::string frame_id_sonar = "sonar";

    // Aria library dependent variables
    ArRobotConnector *conn;
    // ArLaserConnector *laserConnector;
    ArRobot *robot;
    nav_msgs::msg::Odometry position;
    // ros2aria::msg::BumperState bumpers;
    ArPose pos;
    ArFunctorC<Ros2AriaNode> myPublishCB;
    // ArRobot::ChargeState batteryCharge;

    // Robot Calibration Parameters
    int TicksMM, DriftFactor, RevCount;
    float trans_accel, trans_decel, lat_accel, lat_decel, rot_accel, rot_decel;
};

int Ros2AriaNode::Setup()
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

    if (debug_aria)
    {
        // turn on all ARIA debugging
        args->add("-robotLogPacketsReceived");    // log received packets
        args->add("-robotLogPacketsSent");        // log sent packets
        args->add("-robotLogVelocitiesReceived"); // log received velocities
        args->add("-robotLogMovementSent");
        args->add("-robotLogMovementReceived");
        ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
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

    updateParameters();

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

    // // subscribe to command topics
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Ros2AriaNode::cmdvel_cb, this, std::placeholders::_1));

    // register a watchdog for cmd_vel timeout
    double cmdvel_timeout_param = 0.6;
    cmd_vel_timeout = rclcpp::Duration::from_seconds(cmdvel_timeout_param);
    if (cmdvel_timeout_param > 0.0)
        cmdvel_watchdog_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Ros2AriaNode::cmdvel_watchdog, this));

    RCLCPP_INFO(this->get_logger(), "ros2aria: Setup complete");
    return 0;
}

bool Ros2AriaNode::enable_motors_cb(const std_srvs::srv::Empty::Request &request, const std_srvs::srv::Empty::Response &response)
{
    RCLCPP_INFO(this->get_logger(), "Ros2Aria: Enable motors request.");
    robot->lock();
    if (robot->isEStopPressed())
        RCLCPP_WARN(this->get_logger(), "RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
    robot->enableMotors();
    robot->unlock();
    // todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}

bool Ros2AriaNode::disable_motors_cb(const std_srvs::srv::Empty::Request &request, const std_srvs::srv::Empty::Response &response)
{
    RCLCPP_INFO(this->get_logger(), "Ros2Aria: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
    // todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}

void Ros2AriaNode::cmdvel_cb(const geometry_msgs::msg::Twist &msg)
{
    veltime = this->now();
    RCLCPP_INFO(this->get_logger(), "New speed: [%0.2f,%0.2f](%0.3f)", msg.linear.x * 1e3, msg.angular.z, veltime.seconds());
    robot->lock();
    robot->enableMotors();
    robot->unlock();

    robot->lock();
    robot->setVel(msg.linear.x * 1e3);
    if (robot->hasLatVel())
        robot->setLatVel(msg.linear.y * 1e3);
    robot->setRotVel(msg.angular.z * 180 / M_PI);
    robot->unlock();
    RCLCPP_DEBUG(this->get_logger(), "Ros2Aria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.seconds(),
                 (double)msg.linear.x * 1e3, (double)msg.linear.y * 1e3, (double)msg.angular.z * 180 / M_PI);
}

void Ros2AriaNode::spin()
{
    rclcpp::spin(shared_from_this());
}

void Ros2AriaNode::publish()
{
    // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' should not be locked or unlocked.
    pos = robot->getPose();

    tf2::Quaternion q;
    q.setRPY(0, 0, pos.getTh() * M_PI / 180);
    position.pose.pose.position.x = pos.getX() / 1000.0; // Aria returns pose in mm.
    position.pose.pose.position.y = pos.getY() / 1000.0;
    position.pose.pose.position.z = 0.0;
    position.pose.pose.orientation = tf2::toMsg(q);

    position.twist.twist.linear.x = robot->getVel() / 1000.0; // Aria returns velocity in mm/s.
    position.twist.twist.linear.y = robot->getLatVel() / 1000.0;
    position.twist.twist.angular.z = robot->getRotVel() * M_PI / 180;

    position.header.stamp = this->now();
    position.header.frame_id = frame_id_odom;
    position.child_frame_id = frame_id_base_link;
    pose_pub_->publish(position);

    // RCLCPP_DEBUG(this->get_logger(), "Ros2Aria: publish: (time %d) pose x: %d, pose y: %d, pose angle: %d; linear vel x: %d, vel y: %d; angular vel z: %d",
    //              position.header.stamp.sec,
    //              (double)position.pose.pose.position.x,
    //              (double)position.pose.pose.position.y,
    //              (double)position.pose.pose.orientation.w,
    //              (double)position.twist.twist.linear.x,
    //              (double)position.twist.twist.linear.y,
    //              (double)position.twist.twist.angular.z);

    // publishing transform odom->base_link
    odom_trans.header.stamp = this->now();
    odom_trans.header.frame_id = frame_id_odom;
    odom_trans.child_frame_id = frame_id_base_link;

    odom_trans.transform.translation.x = pos.getX() / 1000;
    odom_trans.transform.translation.y = pos.getY() / 1000;
    odom_trans.transform.translation.z = 0.0;
    tf2::Quaternion odom_q;
    odom_q.setRPY(0.0, 0.0, pos.getTh() * M_PI / 180);
    odom_trans.transform.rotation = tf2::toMsg(odom_q);
    odom_broadcaster.sendTransform(odom_trans);

    // int stall = robot->getStallValue();

    // publish recharge state if changed
    char s = robot->getChargeState();
    if (s != recharge_state.data)
    {
        RCLCPP_INFO(this->get_logger(), "Ros2Aria: publishing new recharge state %d.", s);
        recharge_state.data = s;
        recharge_state_pub_->publish(recharge_state);
    }

    // publish motors state if changed
    bool e = robot->areMotorsEnabled();
    if (e != motors_state.data || !published_motors_state)
    {
        RCLCPP_INFO(this->get_logger(), "Ros2Aria: publishing new motors state %d.", e);
        motors_state.data = e;
        motors_state_pub_->publish(motors_state);
        published_motors_state = true;
    }
}

void Ros2AriaNode::updateParameters()
{
    robot->lock();
    // Update all the parameters
    TicksMM = robot->getOrigRobotConfig()->getTicksMM();
    RCLCPP_INFO(this->get_logger(), "This robot's TicksMM parameter: %d", TicksMM);
    robot->comInt(93, TicksMM);
    DriftFactor = robot->getOrigRobotConfig()->getDriftFactor();
    RCLCPP_INFO(this->get_logger(), "This robot's DriftFactor parameter: %d", DriftFactor);
    robot->comInt(89, DriftFactor);
    RevCount = robot->getOrigRobotConfig()->getRevCount();
    RCLCPP_INFO(this->get_logger(), "This robot's RevCount parameter: %d", RevCount);
    robot->comInt(88, RevCount);

    robot->unlock();
}

void Ros2AriaNode::cmdvel_watchdog()
{
    // stop robot if no cmd_vel message was received for 0.6 seconds
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
        RCLCPP_FATAL(node->get_logger(), "Ros2Aria: ROS2 Node Setup Failed !!!");
        return -1;
    }

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Ros2Aria: Exiting !!!");

    return 0;
}