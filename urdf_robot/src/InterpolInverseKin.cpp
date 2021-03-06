#include <ros/ros.h>
#include "urdf_robot/MoveMessage.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include "urdf_robot/InverseKinematics.h"

/* LinearInterpolation class definition */

enum Coord {
  X, Y, Z
};


class LinearInterpolation {
private:
  double x0, y0, z0;
  double x1, y1, z1;
  double duratRever;
public:
  void setInitPos(double initX, double initY, double initZ);
  double interpolate(Coord coo, double time);
  bool isFinished(double time);
  void setNewTarget(double xEnd, double yEnd, double zEnd, double duratR);
};


void LinearInterpolation::setInitPos(double initX, double initY, double initZ) {
  x1 = initX;
  y1 = initY;
  z1 = initZ;
}


double LinearInterpolation::interpolate(Coord coo, double time) {
  switch(coo) {
    case X: return x0 + (x1 - x0) * duratRever * time;
    case Y: return y0 + (y1 - y0) * duratRever * time;
    case Z: return z0 + (z1 - z0) * duratRever * time;
    default: return 0.0;
  }
}


bool LinearInterpolation::isFinished(double time) {
  if(time * duratRever >= 1)
    return true;
  return false;
}

void LinearInterpolation::setNewTarget(double xEnd, double yEnd, double zEnd, double duratR) {
  x0 = x1;
  y0 = y1;
  z0 = z1;
  x1 = xEnd;
  y1 = yEnd;
  z1 = zEnd;
  duratRever = duratR;
}

/* end */

ros::ServiceClient clientKDL;
ros::Timer timer;
ros::Publisher publisher;
ros::Publisher publisherJoints;
ros::Subscriber subscriber;
geometry_msgs::PoseStamped poseStamped;
sensor_msgs::JointState state;
LinearInterpolation linearInterpolation;

double t;
double kwant = 0.04;

const double initX = 2.3;
const double initY = 2.7;
const double initZ = -1.7;

void timer_cb(const ros::TimerEvent& event) {
  poseStamped.pose.position.x = linearInterpolation.interpolate(X, t);
  poseStamped.pose.position.y = linearInterpolation.interpolate(Y, t);
  poseStamped.pose.position.z = linearInterpolation.interpolate(Z, t);

  publisher.publish(poseStamped);

  urdf_robot::InverseKinematics srv;
  srv.request.final_pos.x = linearInterpolation.interpolate(X, t);
  srv.request.final_pos.y = linearInterpolation.interpolate(Y, t);
  srv.request.final_pos.z = linearInterpolation.interpolate(Z, t);
  srv.request.theta1_init = state.position[0];
  srv.request.theta2_init = state.position[1];
  srv.request.d3_init = state.position[2];

  if(clientKDL.call(srv)) {
  	state.position[0] = srv.response.theta1;
  	state.position[1] = srv.response.theta2;
  	state.position[2] = srv.response.d3;
  	publisherJoints.publish(state);
  }

  if(linearInterpolation.isFinished(t))
    timer.stop();
  t += kwant;
}

void subscriber_cb(const urdf_robot::MoveMessageConstPtr &msg) {
  linearInterpolation.setNewTarget(msg->val1, msg->val2, msg->val3, msg->durationRever);

  t = 0.0;
  timer.start();
}

void init_poseStamped() {
  poseStamped.header.frame_id = "/base_link";
  poseStamped.pose.position.x = initX;
  poseStamped.pose.position.y = initY;
  poseStamped.pose.position.z = initZ;  
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
  srv.request.final_pos.x = initX;
  srv.request.final_pos.y = initY;
  srv.request.final_pos.z = initZ;

  if(clientKDL.call(srv)) {
    state.position.push_back(srv.response.theta1);
    state.position.push_back(srv.response.theta2);
    state.position.push_back(srv.response.d3);

    publisherJoints.publish(state);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "InterpolInverseKin");

  ros::NodeHandle n;
  publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  publisherJoints = n.advertise<sensor_msgs::JointState>("calculated_trajectory", 1);
  subscriber = n.subscribe<urdf_robot::MoveMessage>("move_topic", 100, subscriber_cb);
  clientKDL = n.serviceClient<urdf_robot::InverseKinematics>("SolverKDLDanger");

  linearInterpolation.setInitPos(initX, initY, initZ);

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