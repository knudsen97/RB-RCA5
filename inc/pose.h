#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <array>

#ifndef POSE_H
#define POSE_H

class pose
{    
public:


    pose();

    void poseCallback(ConstPosesStampedPtr &msg);
    double getPose(int i);

    enum dataType {x, y, z, w, thetaX, thetaY, thetaZ};

private:
    std::array<double, 6> data;
};

#endif