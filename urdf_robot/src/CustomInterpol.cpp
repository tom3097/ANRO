#include <ros/ros.h>
#include "urdf_robot/DeltaMessage.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include "urdf_robot/InverseKinematics.h"
#include <kdl/kdl.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <cmath>

/* own interpolation */

enum Coord {
  X, Y, Z
};

class VelocityProfile_Custom {
private:
  double tp;
  double tj;
  double a;
  double begin;
public:
  void init(double _tp, double _tj, double a, double _begin);
  double Pos(double time);
};

void VelocityProfile_Custom::init(double _tp, double _tj, double _a, double _begin) {
  tp = _tp;
  tj = _tj;
  a = _a;
  begin = _begin;
}

double VelocityProfile_Custom::Pos(double time) {
  if(time <= tp)
    return begin + a*time*time/2;
  if(time < tp+tj)
    return begin + a*tp*tp/2 + a*tp*(time-tp);
  else
    return begin + a*tp*tp/2 + a*tp*tj + a*tp * (time-tp-tj) - a*(time-tp-tj)*(time-tp-tj)/2;
}

class CustomInterpolation {
private:
  VelocityProfile_Custom x;
  VelocityProfile_Custom y;
  VelocityProfile_Custom z;
  double lastX, lastY, lastZ;
  double max_time;
public:
  void init(double vmax, double amax, double deltaX, double deltaY, double deltaZ);
  double interpolate(Coord coo, double time);
  void setInitPos(double initX, double initY, double initZ);
  bool isFinished(double time);
};

void CustomInterpolation::init(double vmax, double amax, double deltaX, double deltaY, double deltaZ) {
  struct helper {
    VelocityProfile_Custom *c;
    double delta;
    double last;
  };

  helper h_x; h_x.c = &x; h_x.delta = deltaX; h_x.last = lastX;
  helper h_y; h_y.c = &y; h_y.delta = deltaY; h_y.last = lastY;
  helper h_z; h_z.c = &z; h_z.delta = deltaZ; h_z.last = lastZ;

  helper *max_delta = &h_x;
  if(std::abs(h_y.delta) > std::abs(max_delta->delta))
    max_delta = &h_y;
  if(std::abs(h_z.delta) > std::abs(max_delta->delta))
    max_delta = &h_z;

  double _tp, _tj;

  if(vmax*vmax/amax < std::abs(max_delta->delta)) {
    _tp = vmax/ amax;
    _tj = (std::abs(max_delta->delta) - vmax*vmax/amax)/vmax;
    if(max_delta->delta < 0) {
      max_delta->c->init(_tp, _tj, -1*amax, max_delta->last);
    }
    else {
      max_delta->c->init(_tp, _tj, amax, max_delta->last);
    }
  }
  else {
    _tp = std::sqrt(std::abs(max_delta->delta)/amax);
    _tj = 0.0;
    if(max_delta->delta < 0) {
      max_delta->c->init(_tp, 0.0, -1 * amax, max_delta->last);
    }
    else {
      max_delta->c->init(_tp, 0.0, amax, max_delta->last);
    }
  }

  if(max_delta != &h_x) {
    double _a = h_x.delta / (_tp*_tp + _tp*_tj);
    h_x.c->init(_tp, _tj, _a, h_x.last);
  }
  if(max_delta != &h_y) {
    double _a = h_y.delta / (_tp*_tp + _tp*_tj);
    h_y.c->init(_tp, _tj, _a, h_y.last);
  }
  if(max_delta != &h_z) {
    double _a = h_z.delta / (_tp*_tp + _tp*_tj);
    h_z.c->init(_tp, _tj, _a, h_z.last);
  }

  max_time = _tp + _tj + _tp;
}

double CustomInterpolation::interpolate(Coord coo, double time) {
  switch(coo) {
    case X: lastX = x.Pos(time); return lastX;
    case Y: lastY = y.Pos(time); return lastY;
    case Z: lastZ = z.Pos(time); return lastZ;
    default: return 0.0;
  }
}

void CustomInterpolation::setInitPos(double initX, double initY, double initZ) {
  lastX = initX;
  lastY = initY;
  lastZ = initZ;
}

bool CustomInterpolation::isFinished(double time) {
  if(time >= max_time) return true;
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
CustomInterpolation customInterpolation;

double t;
double kwant = 0.04;

const double initX = 2.3;
const double initY = 2.7;
const double initZ = -1.7;

void timer_cb(const ros::TimerEvent& event) {
  poseStamped.pose.position.x = customInterpolation.interpolate(X, t);
  poseStamped.pose.position.y = customInterpolation.interpolate(Y, t);
  poseStamped.pose.position.z = customInterpolation.interpolate(Z, t);

  publisher.publish(poseStamped);

  urdf_robot::InverseKinematics srv;
  srv.request.final_pos.x = customInterpolation.interpolate(X, t);
  srv.request.final_pos.y = customInterpolation.interpolate(Y, t);
  srv.request.final_pos.z = customInterpolation.interpolate(Z, t);

  srv.request.theta1_init = state.position[0];
  srv.request.theta2_init = state.position[1];
  srv.request.d3_init = state.position[2];

  if(clientKDL.call(srv)) {
    state.position[0] = srv.response.theta1;
    state.position[1] = srv.response.theta2;
    state.position[2] = srv.response.d3;
    publisherJoints.publish(state);
  }

  if(customInterpolation.isFinished(t))
    timer.stop();
  t += kwant;
}

void subscriber_cb(const urdf_robot::DeltaMessageConstPtr &msg) {
  customInterpolation.init(msg->vmax, msg->amax, msg->deltaX, msg->deltaY, msg->deltaZ);

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
  ros::init(argc, argv, "CustomInterpol");

  ros::NodeHandle n;
  publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  publisherJoints = n.advertise<sensor_msgs::JointState>("calculated_trajectory", 1);
  subscriber = n.subscribe<urdf_robot::DeltaMessage>("move_topic", 100, subscriber_cb);
  clientKDL = n.serviceClient<urdf_robot::InverseKinematics>("SolverKDL");

  customInterpolation.setInitPos(initX, initY, initZ);

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
