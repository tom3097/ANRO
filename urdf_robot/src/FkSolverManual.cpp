#include <ros/ros.h>
#include <math.h>
#include "urdf_robot/ForwardKinematics.h"
#include <iostream>

bool calculatePositionManual(urdf_robot::ForwardKinematics::Request &req,
                             urdf_robot::ForwardKinematics::Response &res) {
    double sinTheta1 = sin(req.theta1);
    double cosTheta1 = cos(req.theta1);
    double sinTheta2 = sin(req.theta2);
    double cosTheta2 = cos(req.theta2);

    res.final_pos.x = 3*cosTheta1 + 1.5*(cosTheta1*cosTheta2 - sinTheta1*sinTheta2);
    res.final_pos.y = 3*sinTheta1 + 1.5*(sinTheta1*cosTheta2 + cosTheta1*sinTheta2);
    res.final_pos.z = -1*req.d3;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "FkSolverManual");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("SolverManual", calculatePositionManual);
    ros::spin();

    return 0;
}

