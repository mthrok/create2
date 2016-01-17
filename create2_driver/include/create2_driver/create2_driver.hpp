#ifndef CREATE2_DRIVER_CREATE2_DRIVER_HPP
#define CREATE2_DRIVER_CREATE2_DRIVER_HPP

#include "ros/ros.h"
#include "create2_driver/create2.hpp"

#include <create2_msgs/Control.h>

class Create2Driver {
  create2::Create2 c2_;
  ros::NodeHandle nPub_;
  ros::NodeHandle nSub_;

  ros::Publisher pub_;
  ros::Subscriber sub_;

public:
  Create2Driver();
  void init();
  void run();
  void callback(const create2_msgs::Control::ConstPtr& msg);
}; // Create2Driver

#endif // CREATE2_DRIVER_CREATE2_DRIVER_HPP
