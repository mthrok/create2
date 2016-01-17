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

void Create2JoystickControl::callback
    (const sensor_msgs::JoyConstPtr& msg){
  bool send = false;
  create2_msgs::Control ctrl;
  if (msg->buttons[SELECT]) {
    ctrl.mode = create2_msgs::Control::PASSIVE;
    send = true;
    ROS_DEBUG_STREAM("PASSIVE mode");
  }
  if (msg->buttons[PAIRING]) {
    ctrl.mode = create2_msgs::Control::SAFE;
    send = true;
    ROS_DEBUG_STREAM("SAFE mode");
  }
  if (msg->buttons[START]) {
    ctrl.mode = create2_msgs::Control::FULL;
    send = true;
    ROS_DEBUG_STREAM("FULL mode");
  }
  if (msg->axes[LEFT_AXIS_FB] || msg->axes[RIGHT_AXIS_FB]) {
    ctrl.mode = create2_msgs::Control::DRIVE;
    ctrl.axis1 = (float)msg->axes[LEFT_AXIS_FB];
    ctrl.axis2 = (float)msg->axes[RIGHT_AXIS_FB];
    send = true;
    ROS_DEBUG_STREAM(
      "DRIVE:" << ctrl.axis1 << ", " << ctrl.axis2);
  }
  if (send) {
    pub_.publish(ctrl);
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
