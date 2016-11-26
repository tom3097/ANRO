#include <ros/ros.h>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "urdf_robot/ForwardKinematics.h"

bool calculatePositionKDL(urdf_robot::ForwardKinematics::Request &req,
                          urdf_robot::ForwardKinematics::Response &res) {
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(3, 0, 0, 0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(1.5, 0, 0, 0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ), KDL::Frame::DH(0, 0, 0, 0)));

    KDL::JntArray j = KDL::JntArray(3);
    j(0) = req.theta1;
    j(1) = req.theta2;
    j(2) = -1 * req.d3;

    KDL::ChainFkSolverPos_recursive solver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::Frame result;

    solver.JntToCart(j, result);

    res.final_pos.x = result.p.x();
    res.final_pos.y = result.p.y();
    res.final_pos.z = result.p.z();

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "FkSolverKDL");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("SolverKDL", calculatePositionKDL);
    ros::spin();

    return 0;
}
