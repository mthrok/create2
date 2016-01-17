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
    "create2/conrtol", 10, &Create2Driver::callback, this);
}

void Create2Driver::run() {
  c2_.start();
  c2_.full();

  ros::Rate loop_rate(10);
  while(ros::ok()) {
    c2_.drive(30, 0);
    pub_.publish(c2_.getStatus());
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Create2Driver::callback
    (const create2_msgs::Control::ConstPtr& msg) {
  std::cout << "Message received." << std::endl;
}

int main(int narg, char* argv[]) {
  ros::init(narg, argv, "create2_driver");
  Create2Driver n;
  n.init();
  n.run();
  return 0;
}
