#include "ros/ros.h"

#include <create2_msgs/RoombaSensors.h>
#include "create2_driver/create2_driver.hpp"

int main(int narg, char* argv[]) {
  ros::init(narg, argv, "create2driver");

  create2::Create2 c2;
  c2.init();
  c2.start();
  c2.full();

  ros::NodeHandle n;
  ros::Publisher create2_state_pub =
    n.advertise<create2_msgs::RoombaSensors>
      ("create2state", 10);

  ros::Rate loop_rate(10);

  while(ros::ok()) {
    c2.drive(30, 0);
    create2_state_pub.publish(c2.getStatus());
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
