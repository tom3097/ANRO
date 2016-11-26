#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include "urdf_robot/ForwardKinematics.h"

static ros::ServiceClient clientKDL;
static ros::ServiceClient clientManual;

static ros::ServiceClient *chosenOne;

static ros::Publisher publisherMarker;
static ros::Subscriber subscriberJoints;

void posChangedCallbackKDL(const sensor_msgs::JointStateConstPtr &msg) {
    urdf_robot::ForwardKinematics srv;
    srv.request.theta1 = msg->position[0];
    srv.request.theta2 = msg->position[1];
    srv.request.d3 = msg->position[2];

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.ns = "final_pos";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.id = 0;

    if(chosenOne == &clientKDL) {
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 0.8;
    }
    else {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        marker.color.a = 0.8;
    }

    if(chosenOne->call(srv)) {
        geometry_msgs::Pose p;
        p.position = srv.response.final_pos;
        marker.pose = p;
        publisherMarker.publish(marker);
    }
    else {
        ROS_ERROR_STREAM("Client can not publish");
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ManInTheMiddle");

    if(argc != 2) {
        ROS_ERROR_STREAM("Usage: ManInTheMiddle [KDL/Manual]");
        return 1;
    }

    std::string solver = std::string(argv[1]);
    if (solver != "KDL" && solver != "Manual") {
        ROS_ERROR_STREAM("Usage: ManInTheMiddle [KDL/Manual]");
        return 1;
    }      

    chosenOne = ("KDL" == solver) ? &clientKDL : &clientManual;

    ros::NodeHandle n;

    publisherMarker = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    clientKDL = n.serviceClient<urdf_robot::ForwardKinematics>("SolverKDL");
    clientManual = n.serviceClient<urdf_robot::ForwardKinematics>("SolverManual");
    subscriberJoints = n.subscribe<sensor_msgs::JointState>("joint_states", 100, posChangedCallbackKDL);

    ros::spin();

    return 0;
}
