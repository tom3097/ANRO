#include <ros/ros.h>
#include "urdf_robot/JointStateInter.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "MovePublisher");

  if(argc != 5) {
    ROS_ERROR_STREAM("Usage: myStatePublisher theta1, theta2, d3, duration");
    return 1;
  }

  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<urdf_robot::JointStateInter>("move_topic", 100);

  double theta1 = strtod(argv[1], NULL);
  double theta2 = strtod(argv[2], NULL);
  double d3 = strtod(argv[3], NULL);
  double duration = strtod(argv[4], NULL);
  double durationRever = 1.0/duration;

  urdf_robot::JointStateInter jsi;
  jsi.theta1 = theta1;
  jsi.theta2 = theta2;
  jsi.d3 = d3;
  jsi.durationRever = durationRever;

  ros::Duration(2.0).sleep();
  publisher.publish(jsi);
  ros::Duration(duration+0.5).sleep();

  ros::spinOnce();

  return 0;
}
