#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <cmath>
#include <iostream>
#include <array>
#include <iomanip>
#include <opencv2/opencv.hpp>


#ifndef LOCATION_H
#define LOCATION_H



class location
{
public:
    location();
    ~location();

    void update(float speed, float intDir, int sec);
    /* update location of robot
     * sec is for debug and later implementation with time
     */

    void particleFilter(cv::Mat& map, ConstLaserScanStampedPtr &msg);
    /* update location of robot
     */

    void plot(cv::Mat& map, float x, float y);
    /* plots the robot position given map and initial position
     * used for debug and visualization
     */

    float get(char pose);
    /* pose can be the following:
     * 'x' for x
     * 'y' for y
     * 't' for theta
     */

private:
    //position and orientation data
    float x;
    float y;
    float theta;
};

#endif