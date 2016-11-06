#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <string>
#include <cmath>
#include <cstring>
#include <iostream>

double a1 = 3;
double a2 = 1.5;
double d3 = 2;
double theta1 = 3.14/3;
double theta2 = 3.14/6;


void print(std::string title, int number, double *array, int size) {
    std::cout << title << " " << number << ":";
    for(int i = 0; i < size; ++i) {
        std::cout << "\t" << array[i];
    }
    std::cout << std::endl;
}

int main() {
    KDL::Chain chain;
    
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a1, 0, 0, theta1)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(a2, M_PI, 0, theta2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0, 0, d3, 0)));
    
    double joints[3];
    double tips[3];
    double rpy[3];
    
    for(int i = 0; i < 3; ++i) {
        memcpy(joints, chain.getSegment(i).getJoint().JointAxis().data, 3*sizeof(double));
        memcpy(tips, chain.getSegment(i).getFrameToTip().p.data, 3*sizeof(double));
        chain.getSegment(i).getFrameToTip().M.GetRPY(rpy[0], rpy[1], rpy[2]);
        
        print("Joint", i, joints, 3);
        print("Tip frame xyz", i, tips, 3);
        print("Tip frame rpy", i, rpy, 3);
    }
    return 0;
}
    
