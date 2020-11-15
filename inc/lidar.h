#ifndef LIDAR_H
#define LIDAR_H

#include <array>

#include "fl/Headers.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>

class lidar
{
public:    
    lidar();
    ~lidar();

    void lidarCallback(ConstLaserScanStampedPtr &msg);

    float getRange(int i) {return range[i];}
    float getAngle(int i) {return angle[i];}

    //void lidarMin(ConstLaserScanStampedPtr &msg);
    //lidarData lidarMin(ConstLaserScanStampedPtr &msg);

private:
    std::array<float, 640> range;
    std::array<float, 640> angle;

// struct lidarData{
//     float range;
//     float angle;
// };

};

#endif // LIDAR_H
