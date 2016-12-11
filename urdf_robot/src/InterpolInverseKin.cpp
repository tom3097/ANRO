#include <ros/ros.h>
#include "urdf_robot/MoveMessage.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include "urdf_robot/InverseKinematics.h"

ros::ServiceClient clientKDL;
ros::Timer timer;
ros::Publisher publisher;
ros::Publisher publisherJoints;
ros::Subscriber subscriber;
geometry_msgs::PoseStamped poseStamped;
sensor_msgs::JointState state;

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
  double getBeginX();
  double getBeginY();
  double getBeginZ();
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

double Grasper::getBeginX() {
  return this->x0;
}

double Grasper::getBeginY() {
  return this->y0;
}

double Grasper::getBeginZ() {
  return this->z0;
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

  urdf_robot::InverseKinematics srv;
  srv.request.final_pos.x = grasper.interpolateX(t);
  srv.request.final_pos.y = grasper.interpolateY(t);
  srv.request.final_pos.z = grasper.interpolateZ(t);
  srv.request.theta1_init = state.position[0];
  srv.request.theta2_init = state.position[1];
  srv.request.d3_init = state.position[2];

  if(clientKDL.call(srv)) {
  	state.position[0] = srv.response.theta1;
  	state.position[1] = srv.response.theta2;
  	state.position[2] = srv.response.d3;
  	publisherJoints.publish(state);
  }
  else {
  	ROS_ERROR_STREAM("Client can not publish");
  	return;
  }

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
  poseStamped.pose.position.x = grasper.getEndX(); 
  poseStamped.pose.position.y = grasper.getEndY(); 
  poseStamped.pose.position.z = grasper.getEndZ();   
  poseStamped.pose.orientation.w = 0; 
  poseStamped.pose.orientation.x = 0; 
  poseStamped.pose.orientation.y = 0; 
  poseStamped.pose.orientation.z = 0;

  publisher.publish(poseStamped);
}

void init_joint_state() {
  state.velocity.push_back(0);
  state.velocity.push_back(0);
  state.velocity.push_back(0);
  state.effort.push_back(0);
  state.effort.push_back(0);
  state.effort.push_back(0);
  state.name.push_back("joint1");
  state.name.push_back("joint2");
  state.name.push_back("joint3");

  urdf_robot::InverseKinematics srv;
  srv.request.final_pos.x = grasper.getEndX();
  srv.request.final_pos.y = grasper.getEndY();
  srv.request.final_pos.z = grasper.getEndZ();

  if(clientKDL.call(srv)) {
    state.position.push_back(srv.response.theta1);
    state.position.push_back(srv.response.theta2);
    state.position.push_back(srv.response.d3);

    publisherJoints.publish(state);
  }
  else {
    ROS_ERROR_STREAM("Client can not publish");
    return;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "InterpolInverseKin");

  ros::NodeHandle n;
  publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  publisherJoints = n.advertise<sensor_msgs::JointState>("calculated_trajectory", 1);
  subscriber = n.subscribe<urdf_robot::MoveMessage>("move_topic", 100, subscriber_cb);
  clientKDL = n.serviceClient<urdf_robot::InverseKinematics>("SolverKDL");

  grasper.setEnd(2.3, 2.7, -1.7);

  ros::Duration(0.4).sleep();
  init_joint_state();
  ros::Duration(1.1).sleep();
  init_poseStamped();
  ros::Duration(0.4).sleep();

  timer = n.createTimer(ros::Duration(kwant), timer_cb);
  timer.stop();

  ros::spin();

  return 0;
}
