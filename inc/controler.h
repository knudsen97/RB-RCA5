#include <opencv2/opencv.hpp>
#include <array>
#include <cmath>
#include "inc/pose.h"

class controler
{
public:
    struct pidValue
    {
        float error    = 0;
        float output   = 0;
        float errorSum = 0;
        float errorPre = 0;
        float errorDif = 0;
        float theta    = 0;
    };
public:
    controler(float kp = 1, float ki = 0, float kd = 0);

    void control(cv::Point2f goal, pose& currentPosition, std::array<float, 2>& otput);
    enum value{
        speed,
        dir
    };

    float kp, ki, kd;

private:
    float pid(pidValue preValues);
    cv::Point2f goal;
    pidValue distance;
    pidValue angle;
    float hystAngleError;
    float hystDistanceError;
};

