#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>
#include <iomanip>


#ifndef LOCATION_H
#define LOCATION_H



class location
{
public:

    struct pose{
        float x;
        float y;
        float theta;
        pose& operator=(const pose& k) { x = k.x; y = k.y; theta = k.theta; return *this; }
    };

    location();
    location(pose initialPose);
    ~location();

    enum state{pose_x, pose_y, pose_theta};

    /**
    update location of robot
    sec is for debug and later implementation with time
    */
    pose update(float speed, float intDir, float sec = 1);

    /**
    generation particles on map and return particles
    */
    void initialParticleDistributer(cv::Mat& map, int particleNumber, std::vector<pose>& output);

    /**
    simulate lidaroutput
    */
    void lidarSimulator(cv::Mat& inputMap, cv::Mat& outputMap, location::pose particle, double lidarMaxRange);

    /**
    update location of robot
    page 332
    */
    void particleFilter(cv::Mat& map, std::vector<pose>& input, std::vector<pose>& output, ConstLaserScanStampedPtr &msg);


    /**
    plots the robot position given map and initial position
    used for debug and visualization
     */
    void plot(cv::Mat& map, float x, float y, std::string windowName = "Location");
    void plot(cv::Mat& map, pose input, std::string windowName = "Location");
    void plot(cv::Mat& map, std::vector<pose> inputs, std::string windowName = "Location");


    /**
    @brief Get pose variable
    @note
    'x' for x
    'y' for y
    't' for theta
    @param pose accepts following arguments: 'x' 'y' 't'
     */
    float get(char pose);
    float get(state index);

    void testingFunction(char input);

private:
    //position and orientation data
    pose objectPose{0, 0, 0};
    float x;
    float y;
    float theta;
    float time;
    float pretime;
};

#endif