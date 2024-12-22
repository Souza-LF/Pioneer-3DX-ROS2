#ifdef ADEPT_PKG
#include <Aria.h>
#else
#include <Aria/Aria.h>
#endif

#include "LaserPublisher.h"
#include "ArTimeToROSTime.h"

#include <math.h>

#include <boost/algorithm/string.hpp>

LaserPublisher::LaserPublisher(ArLaser *_l, rclcpp::Node::SharedPtr &_n, bool _broadcast_tf, const std::string &_tf_frame, const std::string &_parent_tf_frame, const std::string &_global_tf_frame) : laserReadingsCB(this, &LaserPublisher::readingsCB), node(_n), laser(_l), tfname(_tf_frame), parenttfname(_parent_tf_frame), globaltfname(_global_tf_frame), broadcast_tf(_broadcast_tf)
{
  assert(_l);
  laser->lockDevice();
  laser->addReadingCB(&laserReadingsCB);
  laser->unlockDevice();
  std::string laserscan_name(laser->getName());
  boost::erase_all(laserscan_name, ".");
  laserscan_name += "_laserscan";
  std::string pointcloud_name(laser->getName());
  boost::erase_all(pointcloud_name, ".");
  pointcloud_name += "_pointcloud";
  laserscan_pub = this->node->create_publisher<sensor_msgs::msg::LaserScan>(laserscan_name, 20);
  pointcloud_pub = this->node->create_publisher<sensor_msgs::msg::PointCloud>(pointcloud_name, 50);

  tf2::Quaternion q;
  if (laser->hasSensorPosition())
  {
    lasertf.setOrigin(tf::Vector3(laser->getSensorPositionX() / 1000.0, laser->getSensorPositionY() / 1000.0, laser->getSensorPositionZ() / 1000.0));
    q.setRPY(0, 0, ArMath::degToRad(laser->getSensorPositionTh()));
  }
  else
  {
    lasertf.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(0, 0, 0);
  }
  lasertf.setRotation(q);

  laserscan.header.frame_id = "laser_frame";
  laserscan.angle_min = ArMath::degToRad(laser->getStartDegrees());
  laserscan.angle_max = ArMath::degToRad(laser->getEndDegrees());
  // laserscan.time_increment = ?
  laserscan.range_min = 0; // laser->getMinRange() / 1000.0;
  laserscan.range_max = laser->getMaxRange() / 1000.0;
  pointcloud.header.frame_id = globaltfname;

  // Get angle_increment of the laser
  laserscan.angle_increment = 0;
  if (laser->canSetIncrement())
  {
    laserscan.angle_increment = laser->getIncrement();
  }
  else if (laser->getIncrementChoice() != NULL)
  {
    laserscan.angle_increment = laser->getIncrementChoiceDouble();
  }
  assert(laserscan.angle_increment > 0);
  laserscan.angle_increment *= M_PI / 180.0;

  // readingsCallbackTime = new ArTime;
}

LaserPublisher::~LaserPublisher()
{
  laser->lockDevice();
  laser->remReadingCB(&laserReadingsCB);
  laser->unlockDevice();
  // delete readingsCallbackTime;
}