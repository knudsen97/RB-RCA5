#include "inc/fuzzycontrol.h"
using namespace fl;

#define PI 3.14159265358979323846264338327950288
#define GOAL_RADIUS 0.55

fuzzyControl::fuzzyControl()
{

}

fuzzyControl::~fuzzyControl()
{

}


fuzzyControl::fuzzyData fuzzyControl::setControl(pose &robotPose, lidar &lidarData, cv::Point goal)
{
    //Get data from lidar:
    std::array<float, 200> obstacleRanges = lidarData.getData(lidar::range);
    std::array<float, 200> obstacleAngles = lidarData.getData(lidar::angle);
    
    //Now we find the range and angle to the closest object:
    float minRange = obstacleRanges[49], minAngle = 0;
    for(int i = 49; i < obstacleRanges.size() - 51; i++)
    {
        if(minRange > obstacleRanges[i])
        {
            minRange = obstacleRanges[i];
            minAngle = obstacleAngles[i];
        }
    }

    std::cout << "minRange: " << minRange << std::endl;
    std::cout << "minAngle: " << minAngle << std::endl;

    //Get data from pose:
    float x = robotPose.getPose(pose::x);
    float y = robotPose.getPose(pose::y);
    float orientation = robotPose.getPose(pose::thetaZ)*PI; //Orientation is multiplied by PI, so the range is -PI to PI.
    
    //Calculate distance and angle to goal point relative to the robot.
    float a_goal = atan2(goal.y-y, goal.x-x);
    float sign = a_goal >= orientation ? 1 : -1;
    float goalAngle = a_goal - orientation;
    float K = -sign * PI * 2;
    goalAngle = (abs(K+goalAngle) < abs(goalAngle)) ? K+goalAngle : goalAngle;
    float goalDist = abs(goal.x-x) + abs(goal.y-y);

    //Start fuzzy engine and read .fll file
    Engine* engine = FllImporter().fromFile("obstacleAvoidance.fll");

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    InputVariable* delta_goal_dist = engine->getInputVariable("goal_distance");
    InputVariable* delta_goal_angle = engine->getInputVariable("goal_angle");
    
    InputVariable* obstacle_range = engine->getInputVariable("ob_range");
    InputVariable* obstacle_angle = engine->getInputVariable("ob_angle");

    OutputVariable* steer = engine->getOutputVariable("mSteer");
    OutputVariable* speed = engine->getOutputVariable("mSpeed");

    delta_goal_dist->setValue(goalDist);
    delta_goal_angle->setValue(goalAngle);
    obstacle_range->setValue(minRange);
    obstacle_angle->setValue(minAngle);
    engine->process();

    fuzzyData controlData;
    if (goalDist < GOAL_RADIUS)
    {
        controlData.speed = 0;
        controlData.steer = 0;
    }
    else if ((goalAngle < -0.3) || (goalAngle > 0.3))
    {
        controlData.speed = 0;
        controlData.steer = steer->getValue();
    }
    else
    {
        controlData.speed = speed->getValue();
        controlData.steer = steer->getValue();
    }


    return controlData;
}