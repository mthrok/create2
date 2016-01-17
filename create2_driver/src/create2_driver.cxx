#include "create2_driver/create2_driver.hpp"

Create2Driver::Create2Driver()
  : c2_()
  , nPub_()
  , nSub_()
  , pub_()
  , sub_()
{}

void Create2Driver::init() {
  c2_.init();
  pub_ = nPub_.advertise<create2_msgs::Status>(
    "create2/status", 10);
  sub_ = nSub_.subscribe(
    "create2/control", 10, &Create2Driver::callback, this);
}

void Create2Driver::run() {
  c2_.start();
  c2_.full();

  ros::Rate loop_rate(10);
  while(ros::ok()) {
    // c2_.drive(30, 0);
    pub_.publish(c2_.getStatus());
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Create2Driver::callback
    (const create2_msgs::Control::ConstPtr& ctrl) {
  switch (ctrl->mode) {
  case create2_msgs::Control::PASSIVE: {
    c2_.passive();
    break;
  }
  case create2_msgs::Control::SAFE: {
    c2_.safe();
    break;
  }
  case create2_msgs::Control::FULL: {
    c2_.full();
    break;
  }
  case create2_msgs::Control::DRIVE: {
    short left = 500 * ctrl->axis1;
    short right = 500 * ctrl->axis2;
    c2_.driveDirect(right, left);
    break;
  }
  default: {
    std::cout << "Action Not implemented." << std::endl;
    c2_.driveDirect(0, 0);
  }
  }
}

int main(int narg, char* argv[]) {
  ros::init(narg, argv, "create2_driver");
  Create2Driver n;
  n.init();
  n.run();
  return 0;
}
