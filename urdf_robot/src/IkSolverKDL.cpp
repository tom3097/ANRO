#include <ros/ros.h>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include "urdf_robot/InverseKinematics.h"

bool calculateJointsKDL(urdf_robot::InverseKinematics::Request &req,
                        urdf_robot::InverseKinematics::Response &res) {
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(3, 0, 0, 0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(1.5, 0, 0, 0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ), KDL::Frame::DH(0, 0, 0, 0)));

    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    Eigen::MatrixXd m = Eigen::MatrixXd::Identity(6,6);
    m(3,3) = m(4,4) = m(5,5) = 1e-6;
    KDL::ChainIkSolverVel_wdls iksolver1v(chain);
    iksolver1v.setWeightTS(m);
    KDL::ChainIkSolverPos_NR iksolver1(chain, fksolver, iksolver1v, 100, 1e-6);

    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray q_init(chain.getNrOfJoints());
    q_init(0) = req.theta1_init;
    q_init(1) = req.theta2_init;
    q_init(2) = req.d3_init;

    KDL::Frame F_dest = KDL::Frame(KDL::Vector(req.final_pos.x, req.final_pos.y, -1 * req.final_pos.z));

    if(iksolver1.CartToJnt(q_init, F_dest, q)) {
        res.theta1 = q(0);
        res.theta2 = q(1);
        res.d3 = q(2);
    }
    else {
        ROS_ERROR_STREAM("Can not solve inverse kinematics");
        return false;
    }

    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "IkSolverKDL");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("SolverKDL", calculateJointsKDL);
    ros::spin();

    return 0;
}
