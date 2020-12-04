#include "inc/controler.h"
#define PI 3.141592653589793238462643383

double relativeAngle(cv::Point2f a, cv::Point2f b, double orientation)
{
    float goal = atan2(a.y-b.y, a.x-b.x);
    float sign = goal >= orientation ? -1 : 1;
    float goalAngle = goal - orientation;
    float K = -sign * PI * 2;
    double returnVal = (abs(K+goalAngle) < abs(goalAngle)) ? (K+goalAngle) : (goalAngle);
    //std::cout << "goalAngle: " << goalAngle << "\tK+goalAngle" << K+goalAngle << '\n';
    return returnVal;
}

double magnitude(cv::Point2f a, cv::Point2f b)
{
    double returnVal = sqrt( pow((a.x-b.x), 2) + pow((a.y - b.y), 2) );
    return returnVal;
}

controler::controler(float _kp, float _ki, float _kd)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void controler::control(cv::Point2f goal, pose& currentPosition, std::array<float, 2>& output)
{
    //if object goal not the same as argument goal, update
    this->goal = (goal != this->goal) ? (goal) : (this->goal);
    
    //update error
    cv::Point2f pos(currentPosition.getPose(pose::x), currentPosition.getPose(pose::y));
    float ang = relativeAngle(goal, pos, currentPosition.getPose(pose::thetaW)*PI);
    this->angle.error = ang;

    float mag = magnitude(goal, pos);
    this->distance.error = mag;


    //control angle
    float pidOutput = pid(this->angle);
    output[controler::value::dir] = pidOutput;

    //wait til inside +- five degree of target
    if( this->angle.error < 5*(PI/180) &&  this->angle.error > -5*(PI/180))
        output[controler::value::speed] = pid(this->distance);
    else
        output[controler::value::speed] = 0;
        

}

float controler::pid(pidValue preValues)
{
    preValues.errorSum += preValues.error;
    preValues.errorDif = preValues.error - preValues.errorPre;
    preValues.output = preValues.error*kp + preValues.errorSum*ki + preValues.errorDif*kd;
    return preValues.output; 
}