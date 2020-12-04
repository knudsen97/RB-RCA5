#include "inc/lidar.h"


lidar::lidar()
{

}

lidar::~lidar()
{

}

void lidar::lidarCallback(ConstLaserScanStampedPtr &msg)
{
    float angle_min = float(msg->scan().angle_min());
    float angle_increment = float(msg->scan().angle_step());

    float range_max = float(msg->scan().range_max());
    int nranges = msg->scan().ranges_size();
    float minAngle = 0;
    float minRange = range_max;
    for(int i = 0; i < nranges; i++)
    {
        data[lidar::range][i] = float(msg->scan().ranges(i));
        data[lidar::angle][i] = angle_min + i * angle_increment; 
    }
}




