#include <ros/ros.h>
#include "urdf_robot/MoveMessage.h"
#include <geometry_msgs/PoseStamped.h>

ros::Timer timer;
ros::Publisher publisher;
ros::Subscriber subscriber;
geometry_msgs::PoseStamped poseStamped;

double t;
double kwant = 0.04;

class Grasper {
public:
  double x0, y0, z0;
  double x1, y1, z1;
  double duratRever;
public:
  double interpolateX(double t);
  double interpolateY(double t);
  double interpolateZ(double t);
  void setBeginning(double x0, double y0, double z0);
  void setEnd(double x1, double y1, double z1);
  void setDurationRever(double duratRever);
  double getDurationRever();
  double getEndX();
  double getEndY();
  double getEndZ();
} grasper;

double Grasper::interpolateX(double t) {
  return x0 + (x1 - x0) * duratRever * t;
}

double Grasper::interpolateY(double t) {
  return y0 + (y1 - y0) * duratRever * t;
}

double Grasper::interpolateZ(double t) {
  return z0 + (z1 - z0) * duratRever * t;
}

void Grasper::setBeginning(double x0, double y0, double z0) {
  this->x0 = x0;
  this->y0 = y0;
  this->z0 = z0;
}

void Grasper::setEnd(double x1, double y1, double z1) {
  this->x1 = x1;
  this->y1 = y1;
  this->z1 = z1;
}

void Grasper::setDurationRever(double duratRever) {
  this->duratRever = duratRever;
}

double Grasper::getDurationRever() {
  return this->duratRever;
}

double Grasper::getEndX() {
  return this->x1;
}

double Grasper::getEndY() {
  return this->y1;
}

double Grasper::getEndZ() {
  return this->z1;
}

void timer_cb(const ros::TimerEvent& event) {
  poseStamped.pose.position.x = grasper.interpolateX(t);
  poseStamped.pose.position.y = grasper.interpolateY(t);
  poseStamped.pose.position.z = grasper.interpolateZ(t);

  publisher.publish(poseStamped);

  if(t * grasper.getDurationRever() >= 1)
    timer.stop();
  t += kwant;
}

void subscriber_cb(const urdf_robot::MoveMessageConstPtr &msg) {
  grasper.setBeginning(grasper.getEndX(), grasper.getEndY(), grasper.getEndZ());
  grasper.setEnd(msg->val1, msg->val2, msg->val3);
  grasper.setDurationRever(msg->durationRever);

  t = 0.0;
  timer.start();
}

void init_poseStamped() {
  poseStamped.header.frame_id = "/base_link";
  poseStamped.pose.position.x = 2; 
  poseStamped.pose.position.y = 2; 
  poseStamped.pose.position.z = 2;   
  poseStamped.pose.orientation.w = 0; 
  poseStamped.pose.orientation.x = 0; 
  poseStamped.pose.orientation.y = 0; 
  poseStamped.pose.orientation.z = 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "InterpolGrasper");

  init_poseStamped();

  ros::NodeHandle n;
  publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  timer = n.createTimer(ros::Duration(kwant), timer_cb);
  subscriber = n.subscribe<urdf_robot::MoveMessage>("move_topic", 100, subscriber_cb);

  ros::spin();

  return 0;
}
