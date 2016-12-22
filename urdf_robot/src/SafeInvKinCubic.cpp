#include <ros/ros.h>
#include "urdf_robot/MoveMessage.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include "urdf_robot/InverseKinematics.h"
#include <kdl/kdl.hpp>
#include <kdl/velocityprofile_spline.hpp>

/* CubicInterpolation class definition */

enum Coord {
  X, Y, Z
};

class CubicInterpolation {
private:
  KDL::VelocityProfile_Spline x;
  KDL::VelocityProfile_Spline y;
  KDL::VelocityProfile_Spline z;
  double lastX, lastY, lastZ;
public:
  CubicInterpolation();
  CubicInterpolation(double initX, double initY, double initZ);
  void setInitPos(double initX, double initY, double initZ);
  bool isFinished(double time);
  void prepareCubicCoeff(double xEnd, double yEnd, double zEnd, double duration);
  double interpolate(Coord coo, double time);
};


CubicInterpolation::CubicInterpolation() {
  x = KDL::VelocityProfile_Spline();
  y = KDL::VelocityProfile_Spline();
  z = KDL::VelocityProfile_Spline();
  lastX = lastY = lastZ = 0.0;
}


CubicInterpolation::CubicInterpolation(double initX, double initY, double initZ) {
  x = KDL::VelocityProfile_Spline();
  y = KDL::VelocityProfile_Spline();
  z = KDL::VelocityProfile_Spline();
  lastX = initX;
  lastY = initY;
  lastZ = initZ;
}


void CubicInterpolation::setInitPos(double initX, double initY, double initZ) {
  lastX = initX;
  lastY = initY;
  lastZ = initZ;
}


void CubicInterpolation::prepareCubicCoeff(double xEnd, double yEnd, double zEnd, double duration) {
  x.SetProfileDuration(lastX, 0, xEnd, 0, duration);
  y.SetProfileDuration(lastY, 0, yEnd, 0, duration);
  z.SetProfileDuration(lastZ, 0, zEnd, 0, duration);
}


double CubicInterpolation::interpolate(Coord coo, double time) {
  switch(coo) {
    case X: lastX = x.Pos(time); return lastX;
    case Y: lastY = y.Pos(time); return lastY;
    case Z: lastZ = z.Pos(time); return lastZ;
    default: return 0.0;
  }
}

bool CubicInterpolation::isFinished(double time) {
  if(time >= x.Duration())
    return true;
  return false;
}

/* end */

ros::ServiceClient clientKDL;
ros::Timer timer;
ros::Publisher publisher;
ros::Publisher publisherJoints;
ros::Subscriber subscriber;
geometry_msgs::PoseStamped poseStamped;
sensor_msgs::JointState state;
CubicInterpolation cubicInterpolation;

double t;
double kwant = 0.04;

const double initX = 2.3;
const double initY = 2.7;
const double initZ = -1.7;

void timer_cb(const ros::TimerEvent& event) {
  poseStamped.pose.position.x = cubicInterpolation.interpolate(X, t);
  poseStamped.pose.position.y = cubicInterpolation.interpolate(Y, t);
  poseStamped.pose.position.z = cubicInterpolation.interpolate(Z, t);

  publisher.publish(poseStamped);

  urdf_robot::InverseKinematics srv;
  srv.request.final_pos.x = cubicInterpolation.interpolate(X, t);
  srv.request.final_pos.y = cubicInterpolation.interpolate(Y, t);
  srv.request.final_pos.z = cubicInterpolation.interpolate(Z, t);
  srv.request.theta1_init = state.position[0];
  srv.request.theta2_init = state.position[1];
  srv.request.d3_init = state.position[2];

  if(clientKDL.call(srv)) {
    state.position[0] = srv.response.theta1;
    state.position[1] = srv.response.theta2;
    state.position[2] = srv.response.d3;
    publisherJoints.publish(state);
  }

  if(cubicInterpolation.isFinished(t))
    timer.stop();
  t += kwant;
}

void subscriber_cb(const urdf_robot::MoveMessageConstPtr &msg) {
  cubicInterpolation.prepareCubicCoeff(msg->val1, msg->val2, msg->val3, msg->duration);

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
  clientKDL = n.serviceClient<urdf_robot::InverseKinematics>("SolverKDL");

  cubicInterpolation.setInitPos(initX, initY, initZ);

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
