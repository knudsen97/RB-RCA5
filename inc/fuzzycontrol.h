#ifndef FUZZYCONTROL_H
#define FUZZYCONTROL_H

#include "fl/Headers.h"
#include "inc/lidar.h"
#include "inc/pose.h"

class fuzzyControl
{

private:


public:
    struct fuzzyData{
        float steer, speed;
    };

    fuzzyControl();
    ~fuzzyControl();

    fuzzyData setControl(pose &robotPose, lidar &lidarData, cv::Point goal);

};

#endif // FUZZYCONTROL_H
