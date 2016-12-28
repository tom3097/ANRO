#include <ros/ros.h>
#include "urdf_robot/DeltaMessage.h"
#include <cmath>

/* rosrun urdf_robot DeltaPublisher 1.0 0.2 -4.8 -5.5 0 */

int main(int argc, char **argv) {

  ros::init(argc, argv, "DeltaPublisher");

  if(argc != 6) {
    ROS_ERROR_STREAM("Usage: myStatePublisher vmax, amax, deltaX, deltaY, deltaZ");
    return 1;
  }

  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<urdf_robot::DeltaMessage>("move_topic", 100);

  urdf_robot::DeltaMessage msg;
  msg.vmax = strtod(argv[1], NULL);
  msg.amax = strtod(argv[2], NULL);
  msg.deltaX = strtod(argv[3], NULL);
  msg.deltaY = strtod(argv[4], NULL);
  msg.deltaZ = strtod(argv[5], NULL);

  double max_delta = msg.deltaX;
  if(std::abs(msg.deltaY) > std::abs(max_delta))
    max_delta = msg.deltaY;
  if(std::abs(msg.deltaZ) > std::abs(max_delta))
    max_delta = msg.deltaZ;

  double duration;

  if(msg.vmax*msg.vmax/msg.amax < std::abs(max_delta)) {
    double _tp = msg.vmax/msg.amax;
    double _tj = (std::abs(max_delta) - msg.vmax*msg.vmax/msg.amax)/msg.vmax;
    duration = _tp + _tj + _tp;
  }
  else {
    double _tp = std::sqrt(std::abs(max_delta)/msg.amax);
    double _tj = 0.0;
    duration = _tp + _tj + _tp;
  }

  ros::Duration(2.0).sleep();
  publisher.publish(msg);
  ros::Duration(duration+0.5).sleep();

  ros::spinOnce();

  return 0;
}