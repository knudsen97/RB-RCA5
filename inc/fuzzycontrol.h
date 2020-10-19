#ifndef FUZZYCONTROL_H
#define FUZZYCONTROL_H

#include "fl/Headers.h"
#include "inc/lidarscanner.h"

class fuzzyControl
{

private:
    struct fuzzyData{
        float steer;
        float speed;
    };
public:
    fuzzyControl();

    fuzzyData setControl(ConstLaserScanStampedPtr &msg);
    //float setControl(ConstLaserScanStampedPtr &msg);

};

#endif // FUZZYCONTROL_H
