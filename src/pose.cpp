#include "inc/pose.h"

pose::pose()
{
    ;
}

void pose::poseCallback(ConstPosesStampedPtr &_msg)
{
    for (int i = 0; i < _msg->pose_size(); i++) 
    {
        if (_msg->pose(i).name() == "pioneer2dx") 
        {
            data[pose::x] = _msg->pose(i).position().x();
            data[pose::y] = _msg->pose(i).position().y();
            data[pose::z] = _msg->pose(i).position().z();
            data[pose::w] = _msg->pose(i).orientation().w();
            data[pose::thetaX] = _msg->pose(i).orientation().x();
            data[pose::thetaY] = _msg->pose(i).orientation().y();
            data[pose::thetaZ] = _msg->pose(i).orientation().z();
        }
    }
}

double pose::getPose(int i)
{
    return data[i];
}

