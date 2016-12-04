#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "urdf_robot/JointStateInter.h"
#include <visualization_msgs/Marker.h>
#include "urdf_robot/ForwardKinematics.h"

ros::Timer timer;
ros::Publisher publisher;
ros::Subscriber subscriber;
ros::Publisher publisherMarker;
ros::ServiceClient clientKDL;
sensor_msgs::JointState state;
visualization_msgs::Marker marker;

int markerId;

double t;
double kwant = 0.04;

class Shift {
public:
  double theta1_b, theta2_b, d3_b;
  double theta1_e, theta2_e, d3_e;
  double duratRever;
public:
  double interpolateTheta1(double t);
  double interpolateTheta2(double t);
  double interpolateD3(double t);
  void setBeginning(double theta1_b, double theta2_b, double d3_b);
  void setEnd(double theta1_e, double theta2_e, double d3_e);
  void setDurationRever(double duratRever);
  double getDurationRever();
} shift;

double Shift::interpolateTheta1(double t) {
  return theta1_b + (theta1_e - theta1_b) * duratRever * t;
}

double Shift::interpolateTheta2(double t) {
  return theta2_b + (theta2_e-theta2_b) * duratRever * t;
}

double Shift::interpolateD3(double t) {
  return d3_b + (d3_e-d3_b) * duratRever * t;
}

void Shift::setBeginning(double theta1_b, double theta2_b, double d3_b) {
  this->theta1_b = theta1_b;
  this->theta2_b = theta2_b;
  this->d3_b = d3_b;
}

void Shift::setEnd(double theta1_e, double theta2_e, double d3_e) {
  this->theta1_e = theta1_e;
  this->theta2_e = theta2_e;
  this->d3_e = d3_e;
}

void Shift::setDurationRever(double duratRever) {
  this->duratRever = duratRever;
}

double Shift::getDurationRever() {
  return this->duratRever;
}

void deleteMarkers() {
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  publisherMarker.publish(marker);
  markerId = 0;
}

void drawMarker(double x, double y, double z) {
  marker.id = markerId++;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  publisherMarker.publish(marker);
}

void timer_cb(const ros::TimerEvent& event) {
  state.position[0] = shift.interpolateTheta1(t);
  state.position[1] = shift.interpolateTheta2(t);
  state.position[2] = shift.interpolateD3(t);

  urdf_robot::ForwardKinematics srv;

  srv.request.theta1 = state.position[0];
  srv.request.theta2 = state.position[1];
  srv.request.d3 = state.position[2];

  if(clientKDL.call(srv)) {
    drawMarker(srv.response.final_pos.x,srv.response.final_pos.y,srv.response.final_pos.z);
  }
  else {
    ROS_ERROR_STREAM("Client can not publish");
    return;
  }

  publisher.publish(state);

  if(t * shift.getDurationRever() >= 1)
    timer.stop();
  t += kwant;
}

void subscriber_cb(const urdf_robot::JointStateInterConstPtr &msg) {

  shift.setBeginning(state.position[0], state.position[1], state.position[2]);
  shift.setEnd(msg->theta1, msg->theta2, msg->d3);
  shift.setDurationRever(msg->durationRever);

  deleteMarkers();

  t = 0.0;
  timer.start();
}

void init_joint_state() {
  state.position.push_back(0);
  state.position.push_back(0);
  state.position.push_back(1.65);
  state.velocity.push_back(0);
  state.velocity.push_back(0);
  state.velocity.push_back(0);
  state.effort.push_back(0);
  state.effort.push_back(0);
  state.effort.push_back(0);
  state.name.push_back("joint1");
  state.name.push_back("joint2");
  state.name.push_back("joint3");
}

void init_marker() {
  marker.header.frame_id = "/base_link";
  marker.ns = "final_pos";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.5;
  marker.color.a = 0.8;
}

int main(int argc, char **argv) {

  init_joint_state();
  init_marker();

  ros::init(argc, argv, "InterpolTrajectory");
  ros::NodeHandle n;

  publisher = n.advertise<sensor_msgs::JointState>("calculated_trajectory", 1);
  timer = n.createTimer(ros::Duration(kwant), timer_cb);
  subscriber = n.subscribe<urdf_robot::JointStateInter>("move_topic", 100, subscriber_cb);
  publisherMarker = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  clientKDL = n.serviceClient<urdf_robot::ForwardKinematics>("SolverKDL");

  ros::spin();

  return 0;
}