#ifndef VISION_H
#define VISION_H

#include <opencv2/opencv.hpp>
#include <gazebo/msgs/msgs.hh>
#include "./inc/vision.h"

class vision
{
public:
    vision();

    float distanceCalculation(float objH);
private:
    void circleDetection( cv::Mat img);



};

#endif // VISION_H
