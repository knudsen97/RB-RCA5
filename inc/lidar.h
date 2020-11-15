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

    //float getRange(int i) {return range[i];}
    //float getAngle(int i) {return angle[i];}

    enum dataType {range, angle, minRange, minAngle};
    //void lidarMin(ConstLaserScanStampedPtr &msg);
    //lidarData lidarMin(ConstLaserScanStampedPtr &msg);

private:
    std::array<std::array<float, 200>, 2> data;
    // std::array<float, 200> range;
    // std::array<float, 200> angle;

// struct lidarData{
//     float range;
//     float angle;
// };

};

#endif // LIDAR_H
