#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>



class camera
{
public:
    camera();

    void cameraCallback(ConstImageStampedPtr &msg);

    cv::Mat getImage();

private:

    void signal();

    void wait();

    cv::Mat image;

    static boost::mutex cameraMutex;



};




#endif // CAMERA_H
