#ifndef FUZZYCONTROL_H
#define FUZZYCONTROL_H

#include "fl/Headers.h"
#include "inc/lidarscanner.h"

class fuzzyControl
{

private:
    struct fuzzyData{
        float steer, speed;
    };

public:
    fuzzyControl();
    ~fuzzyControl();

    //fuzzyData setControl(ConstLaserScanStampedPtr &msg);
    fuzzyData setControl(ConstPosesStampedPtr &_msg);

};

#endif // FUZZYCONTROL_H
