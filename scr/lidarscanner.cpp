#include "inc/lidarscanner.h"


lidarScanner::lidarScanner()
{

}

lidarScanner::lidarData lidarScanner::lidarMin(ConstLaserScanStampedPtr &msg)
{
    lidarScanner::lidarData data;
    float angle_min = float(msg->scan().angle_min());
    float angle_increment = float(msg->scan().angle_step());

    //float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());
    int nranges = msg->scan().ranges_size();
    float minAngle = 0;
    float minRange = range_max;

    for(int i = 0; i < nranges; i++)
    {
        if(float(msg->scan().ranges(i) < minRange))
        {
            minRange = float(msg->scan().ranges(i));
            minAngle = angle_min + i * angle_increment;
        }
    }
    data.range = minRange;
    data.angle = minAngle;

    return data;
}
