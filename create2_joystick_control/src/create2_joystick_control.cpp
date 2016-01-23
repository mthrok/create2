#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/thread.hpp>

#include <sensor_msgs/Joy.h>
#include <create2_msgs/Control.h>

#define SELECT 0
#define START 3
#define PAIRING 16
#define LEFT_AXIS_FB 1
#define RIGHT_AXIS_FB 3

class Create2JoystickControl{
  ros::NodeHandle nPub_;
  ros::NodeHandle nSub_;

  ros::Publisher pub_;
  ros::Subscriber sub_;
public:
  Create2JoystickControl();
  void init();
  void run();
  void callback(const sensor_msgs::JoyConstPtr& msg);
};

Create2JoystickControl::Create2JoystickControl()
  : nSub_()
{}

void Create2JoystickControl::init() {
  pub_ = nPub_.advertise<create2_msgs::Control>
    ("create2/control", 100);
  sub_ = nSub_.subscribe
    ("/joy", 100, &Create2JoystickControl::callback, this);
}

create2_msgs::Control genModeControl(
    const bool passive, const bool safe, const bool full) {
  create2_msgs::Control ctrl;
  if (passive) {
    ctrl.mode = create2_msgs::Control::PASSIVE;
    ROS_DEBUG_STREAM("PASSIVE mode");
  } else if (safe) {
    ctrl.mode = create2_msgs::Control::SAFE;
    ROS_DEBUG_STREAM("SAFE mode");
  } else {
    ctrl.mode = create2_msgs::Control::FULL;
    ROS_DEBUG_STREAM("FULL mode");
  }
  return ctrl;
}

create2_msgs::Control genDriveControl(float left, float right) {
  float threshold = 0.5;
  // Map [0.0, threshold] to 0.0 and [threshold, 1.0] to [0.0 - 1.0]
  float sign_left = (left < 0.0)? -1.0 : 1.0;
  float sign_right = (right < 0.0)? -1.0 : 1.0;
  left = std::abs(left);
  right = std::abs(right);
  left = (left < threshold) ? 0.0 :
    sign_left * (left - threshold) / (1.0 - threshold);
  right = (right < threshold) ? 0.0 :
    sign_right * (right - threshold) / (1.0 - threshold);

  create2_msgs::Control ctrl;
  ctrl.mode = create2_msgs::Control::DRIVE;
  ctrl.left_axis = left;
  ctrl.right_axis = right;
  ROS_DEBUG_STREAM("DRIVE: "
                   << "L = " << ctrl.left_axis << ", "
                   << "R = " << ctrl.right_axis);
  return ctrl;
}

void Create2JoystickControl::callback
    (const sensor_msgs::JoyConstPtr& msg){
  bool passive = msg->buttons[SELECT];
  bool safe = msg->buttons[PAIRING];
  bool full = msg->buttons[START];
  if (passive || safe || full) {
    pub_.publish(genModeControl(passive, safe, full));
    return;
  }
  float left = msg->axes[LEFT_AXIS_FB];
  float right = msg->axes[RIGHT_AXIS_FB];
  if (left || right) {
    pub_.publish(genDriveControl(left, right));
    return;
  }
}

void Create2JoystickControl::run() {
  ros::spin();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "create2_joystick_drive");
  Create2JoystickControl c;
  c.init();
  c.run();
  return 0;
}
