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
    create2_msgs::Status status = c2_.getStatus();
    pub_.publish(status);
    std::cout
      << "Request:"
      << " V_l " << status.request.left_velocity
      << " V_r " << status.request.right_velocity
      << " [mm/s] " << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Create2Driver::callback
    (const create2_msgs::Control::ConstPtr& ctrl) {
  switch (ctrl->mode) {
  case create2_msgs::Control::PASSIVE: {
    c2_.passive();
    return;
  }
  case create2_msgs::Control::SAFE: {
    c2_.safe();
    return;
  }
  case create2_msgs::Control::FULL: {
    c2_.full();
    return;
  }
  case create2_msgs::Control::DRIVE: {
    create2_msgs::Status status = c2_.getStatus();
    float left = 500.0 * ctrl->left_axis;
    float right = 500.0 * ctrl->right_axis;
    // Scale control vector so that its magnitude does not
    // change too sudden.
    float threshold = 30;
    // One idea for making maneuver more smooth will be make
    // this threshold value change dynamically based on request value.
    // It will involve corner case where request velocity is 0
    // So will be left as is for a while.
    float left_req = status.request.left_velocity;
    float right_req = status.request.right_velocity;

    float mag = std::sqrt(left * left + right * right + 1);
    float mag_req = std::sqrt(left_req * left_req + right_req * right_req + 1);
    if (mag - mag_req > threshold) {
      float scale = (mag_req + threshold) / mag;
      ROS_DEBUG_STREAM("Scaling down: "
        << "L: " << left << " -> " << left * scale << ", "
        << "R: " << right << " -> " << right * scale);
      left = left * scale;
      right = right * scale;
    } else if (mag_req - mag > threshold) {
      float scale = (mag_req - threshold) / mag;
      ROS_DEBUG_STREAM("Scaling up: "
        << "L: " << left << " -> " << left * scale << ", "
        << "R: " << right << " -> " << right * scale);
      left = left * scale;
      right = right * scale;
    }
    c2_.driveDirect(right, left);
    return;
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
