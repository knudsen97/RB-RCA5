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
    // for(int i = 0; i < obstacleRanges.size() - 1; i++)
    // {
    //     if(minRange > obstacleRanges[i])
    //     {
    //         minRange = obstacleRanges[i];
    //         minAngle = obstacleAngles[i];
    //     }
    // }

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

    // std::cout << "a_goal      : " << a_goal <<  std::endl;
    // std::cout << "sign        : " << sign << std::endl;
    // std::cout << "K           : " << K << std::endl;
    // std::cout << "Goal angle  : " << goalAngle << std::endl;
    //std::cout << "Goal dist   : " << goalDist << std::endl;
    // std::cout << "orientation : " << orientation << std::endl;

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

    // if(goalDist < 1.0)
    // {
    //     controlData.speed = (goalAngle < 0.3 && goalAngle > -0.3) ? speed->getValue() : 0;
    //     controlData.steer = (goalAngle == 0) ? 0 : steer->getValue();
    // }

// fuzzyControl::fuzzyData fuzzyControl::setControl(ConstLaserScanStampedPtr &msg)
// {
//     fuzzyControl::fuzzyData controlData;

//     float angle, range, x, y;

//     lidarScanner A;
//     range = A.lidarMin(msg).range;
//     angle = A.lidarMin(msg).angle;

//     //Local obstacle avoidance 1st iteration
//     //Engine* engine = FllImporter().fromFile("obstacleAvoidance.fll");

//     //Local obstacle avoidance 2nd iteration
//     //Engine* engine = FllImporter().fromFile("obstacleAvoidance2.fll");
    
//     //Move to point 1st. iteration
//     //Engine* engine = FllImporter().fromFile("obstacleAvoidance3.fll");

//     //Local obstacle avoidance 3rd iteration
//     Engine* engine = FllImporter().fromFile("obstacleAvoidance4.fll");

//     std::string status;
//     if (not engine->isReady(&status))
//         throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

//     InputVariable* obstacle_range = engine->getInputVariable("obstacle_range");
//     InputVariable* obstacle_angle = engine->getInputVariable("obstacle_angle");

//     OutputVariable* steer = engine->getOutputVariable("mSteer");
//     OutputVariable* speed = engine->getOutputVariable("mSpeed");


//     //std::cout << range << std::endl;
//     obstacle_range->setValue(range);
//     obstacle_angle->setValue(angle);
//     engine->process();

//     controlData.speed = speed->getValue();
//     controlData.steer = steer->getValue();

//     return controlData;
// }


//  fuzzyControl::fuzzyData fuzzyControl::setControl(ConstPosesStampedPtr &_msg)
//  {
//     fuzzyControl::fuzzyData A;
//     double x_goal = -6.0f;
//     double y_goal = 4.0f;
   
//     double x = _msg->pose(0).position().x();
//     double y = _msg->pose(0).position().y();
//     double orientation = _msg->pose(0).orientation().z() * PI;
    
//     double a_goal = atan2(y_goal-y, x_goal-x);

//     double sign = a_goal > orientation ? 1 : -1;
//     double angle = a_goal - orientation;
//     double K = -sign * PI * 2;
//     angle = (abs(K+angle) < abs(angle)) ? K+angle : angle;
//     double dist = abs(x_goal-x)+abs(y_goal-y);

//     std::cout << "angle diff: " << angle << "\ndist: " << dist << "\norientation: " << orientation<< "\natan2: "<< atan2(y_goal-y, x_goal-x) <<  std::endl;

//     Engine* engine = FllImporter().fromFile("obstacleAvoidance3.fll");

//     std::string status;
//     if (not engine->isReady(&status))
//         throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

//     InputVariable* dist_diff = engine->getInputVariable("dist_diff");
//     InputVariable* angle_diff = engine->getInputVariable("angle_diff");

//     OutputVariable* steer = engine->getOutputVariable("mSteer");
//     OutputVariable* speed = engine->getOutputVariable("mSpeed");

//     dist_diff->setValue(dist);
//     angle_diff->setValue(angle);
//     engine->process();


//     if(dist > GOAL_RADIUS)
//     {
//         A.speed = (angle < 0.3 && angle > -0.3) ? speed->getValue() : 0;
//         A.steer = steer->getValue();
//     }
//     else
//     {
//         A.speed = 0;
//         A.steer = 0;
//     }
    

//     return A;
//  }


