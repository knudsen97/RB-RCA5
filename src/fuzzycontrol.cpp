#include "inc/fuzzycontrol.h"
using namespace fl;

fuzzyControl::fuzzyControl()
{

}

fuzzyControl::fuzzyData fuzzyControl::setControl(ConstLaserScanStampedPtr &msg)
{
    fuzzyControl::fuzzyData controlData;
    float angle, range;

    lidarScanner A;
    range = A.lidarMin(msg).range;
    angle = A.lidarMin(msg).angle;

    Engine* engine = FllImporter().fromFile("../control/obstacleAvoidance.fll");
    //Engine* engine = FllImporter().fromFile("obstacleAvoidance.fll");

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    InputVariable* obstacle_range = engine->getInputVariable("obstacle_range");
    InputVariable* obstacle_angle = engine->getInputVariable("obstacle_angle");

    OutputVariable* steer = engine->getOutputVariable("mSteer");
    OutputVariable* speed = engine->getOutputVariable("mSpeed");


    //std::cout << range << std::endl;
    obstacle_range->setValue(range);
    obstacle_angle->setValue(angle);
    engine->process();

    controlData.speed = speed->getValue();
    controlData.steer = steer->getValue();

    return controlData;
}


