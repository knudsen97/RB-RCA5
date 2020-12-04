#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <array>
#include <mutex>
#include <cmath>


#ifndef POSE_H
#define POSE_H

class pose
{    
public:


    pose();

    void poseCallback(ConstPosesStampedPtr &msg);
    double getPose(int i);

    enum dataType {x, y, z, thetaX, thetaY, thetaZ};
    /**
     * the angles range is [-pi, pi]
     */

private:
    void angleCal(double w, double x, double y, double z);
    std::array<double, 6> data;
    //std::mutex poseMutex;
};

#endif

