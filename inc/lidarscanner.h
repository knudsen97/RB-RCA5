#ifndef LIDARSCANNER_H
#define LIDARSCANNER_H

#include "fl/Headers.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

class lidarScanner
{
private:
    struct lidarData{
        float range;
        float angle;
    };

public:    
    lidarScanner();
    lidarData lidarMin(ConstLaserScanStampedPtr &msg);

};

#endif // LIDARSCANNER_H
