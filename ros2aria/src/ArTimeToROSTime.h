#ifndef ARTIMETOROSTIMESTAMP_H
#define ARTIMETOROSTIMESTAMP_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>

#ifdef ADEPT_PKG
#include "ariaUtil.h"
#else
#include "Aria/ariaUtil.h"
#endif

inline rclcpp::Time convertArTimeToROS2(const ArTime &t, const rclcpp::Node::SharedPtr &node)
{
    // ARIA/ARNL times are in reference to an arbitrary starting time, not OS
    // clock, so find the time elapsed between now and t
    // to adjust the time stamp in ROS2 time vs. now accordingly.
    ArTime arianow;
    const double dtsec = static_cast<double>(t.mSecSince(arianow)) / 1000.0;
    rclcpp::Time current_time = node->now();
    rclcpp::Duration dt = rclcpp::Duration::from_seconds(dtsec);
    return current_time - dt;
}

#endif