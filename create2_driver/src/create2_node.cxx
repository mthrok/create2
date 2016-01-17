#include "ros/ros.h"

#include <create2_msgs/Status.h>
#include <create2_msgs/Control.h>
#include "create2_driver/create2_driver.hpp"

class Create2DriverNode {
  create2::Create2 c2_;
  ros::NodeHandle nPub_;
  ros::NodeHandle nSub_;

  ros::Publisher pub_;
  ros::Subscriber sub_;

public:
  Create2DriverNode();
  void init();
  void run();
  void controlCallback(const create2_msgs::Control::ConstPtr& msg);
};

Create2DriverNode::Create2DriverNode()
  : c2_()
  , nPub_()
  , nSub_()
  , pub_()
  , sub_()
{}

void Create2DriverNode::init() {
  c2_.init();
}

void Create2DriverNode::run() {
  c2_.start();
  c2_.full();

  sub_ = nSub_.subscribe("create2control", 10, &Create2DriverNode::controlCallback, this );
  pub_ = nPub_.advertise<create2_msgs::Status>("create2status", 10);

  ros::Rate loop_rate(10);
  while(ros::ok()) {
    c2_.drive(30, 0);
    pub_.publish(c2_.getStatus());
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Create2DriverNode::controlCallback(const create2_msgs::Control::ConstPtr& msg) {
  std::cout << "Message received." << std::endl;
}

int main(int narg, char* argv[]) {
  ros::init(narg, argv, "create2driver");
  Create2DriverNode n;
  n.init();
  n.run();
  return 0;
}
