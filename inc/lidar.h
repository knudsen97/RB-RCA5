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

    std::array<float, 200> getData(int i) {return data[i];}
    
    enum dataType {range, angle, minRange, minAngle};

private:
    std::array<std::array<float, 200>, 2> data;

};

#endif // LIDAR_H
