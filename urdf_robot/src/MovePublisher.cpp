#include <ros/ros.h>
#include "urdf_robot/MoveMessage.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "MovePublisher");

  if(argc != 5) {
    ROS_ERROR_STREAM("Usage: myStatePublisher val1, val2, val3, duration");
    return 1;
  }

  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<urdf_robot::MoveMessage>("move_topic", 100);

  urdf_robot::MoveMessage msg;
  msg.val1 = strtod(argv[1], NULL);
  msg.val2 = strtod(argv[2], NULL);
  msg.val3 = strtod(argv[3], NULL);
  double duration = strtod(argv[4], NULL);
  msg.durationRever = 1.0/duration;

  ros::Duration(2.0).sleep();
  publisher.publish(msg);
  ros::Duration(duration+0.5).sleep();

  ros::spinOnce();

  return 0;
}
