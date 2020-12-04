#include "inc/pose.h"
#define PI 3.1415926535897932384626


pose::pose()
{
    ;
}

void pose::poseCallback(ConstPosesStampedPtr &_msg)
{
//    poseMutex.lock();
    for (int i = 0; i < _msg->pose_size(); i++) 
    {
        if (_msg->pose(i).name() == "pioneer2dx") 
        {
            data[pose::x] = _msg->pose(i).position().x();
            data[pose::y] = _msg->pose(i).position().y();
            data[pose::z] = _msg->pose(i).position().z();
            //convert to auler angles
            angleCal(
                _msg->pose(i).orientation().w(), 
                _msg->pose(i).orientation().x(), 
                _msg->pose(i).orientation().y(), 
                _msg->pose(i).orientation().z()
            );
        }
    }
//    poseMutex.unlock();
}

double pose::getPose(int i)
{
//    poseMutex.lock();
    return data[i];
//    poseMutex.unlock();
}

void pose::angleCal(double w, double x, double y, double z){

// calculate theta_X
double sinr_cosp = 2 * (w * x + y * z);
double cosr_cosp = 1 - 2 * (x * x + y * y);
this->data[pose::thetaX] = std::atan2(sinr_cosp, cosr_cosp);

// calculate theta_Y
double sinp = 2 * (w * y - z * x);
if (std::abs(sinp) >= 1)
    this->data[pose::thetaY] = std::copysign(PI / 2, sinp); // use PI rad if out of range
else
    this->data[pose::thetaY] = std::asin(sinp);

// calculate theta_Z
double siny_cosp = 2 * (w * z + x * y);
double cosy_cosp = 1 - 2 * (y * y + z * z);
this->data[pose::thetaZ] = std::atan2(siny_cosp, cosy_cosp);
};

